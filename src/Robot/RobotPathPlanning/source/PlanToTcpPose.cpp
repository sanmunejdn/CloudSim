/// @file PlanToTcpPose.cpp
/// @brief planToTcpPose 入口：IK 目标 + 关节空间规划 + 双输出

#include "RobotPathPlanning.h"

#include "CollisionValidity.h"
#include "JointSpaceRrtPlanner.h"
#include "OmplJointSpacePlanner.h"
#include "PathPostProcess.h"

#include "ToolKinematics.h"
#include "Adapters.h"
#include "UrdfIkSolverOptions.h"
#include "UrdfNumericalIk.h"
#include "UrdfRobotLoader.h"

#include <cmath>

#include <QHash>
#include <QString>
#include <QVector>

namespace robot_path
{
namespace
{

engine::RigidTransform rigidFromTcpPose(const TcpPose& p)
{
	Eigen::Quaterniond q(p.quatXyzw[3], p.quatXyzw[0], p.quatXyzw[1], p.quatXyzw[2]);
	return engine::RigidTransform::fromTranslationQuat(Eigen::Vector3d(p.transMm[0], p.transMm[1], p.transMm[2]), q);
}

engine::RigidTransform rigidFromBackend(const BackendMat4& m)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
		cm[static_cast<size_t>(i)] = m.v[i];
	return engine::rigidTransformFromColMajor(cm);
}

bool solveGoalJoints(const PlanRequest& req, std::vector<double>& goalQ, std::string& errMsg)
{
	const engine::RigidTransform T_base_target = rigidFromTcpPose(req.goalToolInBase);
	const engine::RigidTransform T_tool = rigidFromBackend(req.T_flange_tool);
	const engine::RigidTransform T_base_flange = engine::flangeFromToolOrigin(T_base_target, T_tool);

	UrdfRobotLoader::UrdfPoseIkTarget target{};
	T_base_flange.translationMm(target.posMm[0], target.posMm[1], target.posMm[2]);
	target.hasOrientation = req.options.useOrientation;
	const Eigen::Quaterniond quat = T_base_flange.rotation();
	target.quatXyzw[0] = quat.x();
	target.quatXyzw[1] = quat.y();
	target.quatXyzw[2] = quat.z();
	target.quatXyzw[3] = quat.w();

	UrdfRobotLoader::UrdfIkSolverOptions opts{};
	std::string ikFail;
	goalQ = UrdfRobotLoader::solveArmPoseDampedLeastSquares(req.urdfPath, req.flangeLinkName, target,
															req.startJointRad, opts, &ikFail);
	if (goalQ.empty())
	{
		errMsg = ikFail.empty() ? "IK failed for goal pose" : ikFail;
		return false;
	}

	// IK 残差过大则拒绝（避免规划“成功”但 TCP 终点错位）
	QHash<QString, engine::RigidTransform> linkWorld;
	QString fkErr;
	QVector<double> qv;
	for (double v : goalQ)
		qv.push_back(v);
	if (!UrdfRobotLoader::computeLinkWorldRigidTransforms(req.urdfPath, qv, linkWorld, &fkErr))
	{
		errMsg = fkErr.isEmpty() ? "FK failed for goal joints" : fkErr.toStdString();
		return false;
	}
	const auto it = linkWorld.constFind(req.flangeLinkName);
	if (it == linkWorld.constEnd())
	{
		errMsg = "goal flange link missing in FK";
		return false;
	}
	const engine::RigidTransform T_got = engine::toolOriginFromFlange(*it, T_tool);
	double gx = 0, gy = 0, gz = 0, hx = 0, hy = 0, hz = 0;
	T_base_target.translationMm(gx, gy, gz);
	T_got.translationMm(hx, hy, hz);
	const double errMm = std::sqrt((gx - hx) * (gx - hx) + (gy - hy) * (gy - hy) + (gz - hz) * (gz - hz));
	constexpr double kMaxIkResidualMm = 5.0;
	if (errMm > kMaxIkResidualMm)
	{
		errMsg = "IK residual too large: " + std::to_string(errMm) + " mm";
		return false;
	}
	return true;
}

} // namespace

bool planToTcpPose(const PlanRequest& req, PathResult& out)
{
	out = PathResult{};
	if (req.urdfPath.isEmpty() || req.flangeLinkName.isEmpty())
	{
		out.errMsg = "missing urdf or flange link";
		return false;
	}
	if (req.startJointRad.empty())
	{
		out.errMsg = "empty start joints";
		return false;
	}
	if (req.options.checkCollision && !req.world)
	{
		out.errMsg = "collision world is null";
		return false;
	}

	detail::JointLimits lim;
	std::string limErr;
	if (!detail::loadJointLimits(req.urdfPath, lim, &limErr))
	{
		out.errMsg = limErr.empty() ? "failed to load joint limits" : limErr;
		return false;
	}

	PlanRequest armReq = req;
	if (armReq.startJointRad.size() > lim.lowerRad.size())
		armReq.startJointRad.resize(lim.lowerRad.size());
	if (armReq.startJointRad.size() != lim.lowerRad.size())
	{
		out.errMsg = "start joint count mismatch (got " + std::to_string(req.startJointRad.size()) + ", urdf " +
					 std::to_string(lim.lowerRad.size()) + ")";
		return false;
	}

	std::vector<double> goalQ;
	if (!solveGoalJoints(armReq, goalQ, out.errMsg))
		return false;

	// 无障碍/直达可行时：关节直线最短且可复现
	if (detail::isStateValid(armReq, lim, armReq.startJointRad) && detail::isStateValid(armReq, lim, goalQ)
		&& detail::isSegmentValid(armReq, lim, armReq.startJointRad, goalQ, armReq.options.longestValidSegmentRad))
	{
		out.jointTrajectoryRad = {armReq.startJointRad, goalQ};
		out.plannerName = "Direct";
		out.ok = true;
	}
	else
	{
		bool planned = false;
		std::string lastErr;
#if defined(CLOUDSIM_HAS_OMPL)
		// BIT*（批量启发，路径长度 anytime）→ Informed RRT* → RRT* → RRTConnect
		// 参考：https://github.com/ompl/ompl demos/OptimalPlanning.cpp
		{
			PlanRequest tryReq = armReq;
			tryReq.options.plannerId = "BITstar";
			tryReq.options.planningTimeSec = std::max(10.0, armReq.options.planningTimeSec);
			planned = detail::planJointSpaceOmpl(tryReq, lim, goalQ, out);
			if (!planned)
				lastErr = out.errMsg;
		}
		if (!planned)
		{
			PlanRequest tryReq = armReq;
			tryReq.options.plannerId = "InformedRRTstar";
			tryReq.options.planningTimeSec = std::max(10.0, armReq.options.planningTimeSec);
			planned = detail::planJointSpaceOmpl(tryReq, lim, goalQ, out);
			if (!planned)
				lastErr = out.errMsg;
		}
		if (!planned)
		{
			PlanRequest tryReq = armReq;
			tryReq.options.plannerId = "RRTstar";
			tryReq.options.planningTimeSec = std::max(8.0, armReq.options.planningTimeSec);
			planned = detail::planJointSpaceOmpl(tryReq, lim, goalQ, out);
			if (!planned)
				lastErr = out.errMsg;
		}
		if (!planned)
		{
			PlanRequest tryReq = armReq;
			tryReq.options.plannerId = "RRTConnect";
			tryReq.options.planningTimeSec = std::max(5.0, armReq.options.planningTimeSec);
			planned = detail::planJointSpaceOmpl(tryReq, lim, goalQ, out);
			if (!planned)
				lastErr = out.errMsg;
		}
#endif
		if (!planned)
		{
			PlanRequest tryReq = armReq;
			tryReq.options.plannerId = "RRTConnect";
			tryReq.options.planningTimeSec = std::max(5.0, armReq.options.planningTimeSec);
			planned = detail::planJointSpaceRrt(tryReq, lim, goalQ, out);
			if (!planned && out.errMsg.empty())
				out.errMsg = lastErr.empty() ? "motion planning failed" : lastErr;
		}
		if (!planned)
			return false;
		detail::shortcutJointPath(armReq, lim, out);
	}

	// 捷径后仍保证末端 = goalQ
	if (!out.jointTrajectoryRad.empty())
	{
		auto& back = out.jointTrajectoryRad.back();
		if (back.size() == goalQ.size())
		{
			double dq = 0.0;
			for (std::size_t i = 0; i < goalQ.size(); ++i)
			{
				const double d = back[i] - goalQ[i];
				dq += d * d;
			}
			dq = std::sqrt(dq);
			if (dq > 1e-3)
			{
				if (!detail::isSegmentValid(armReq, lim, back, goalQ, armReq.options.longestValidSegmentRad))
				{
					out.ok = false;
					out.errMsg = "path end != goal joints after shortcut";
					return false;
				}
				out.jointTrajectoryRad.push_back(goalQ);
			}
			else
			{
				back = goalQ;
			}
		}
	}

	// 写入程序用：加密中间关节样本（去掉起终点后仍有 Pmid）
	detail::densifyJointPath(out, std::max(0.05, armReq.options.longestValidSegmentRad));
	// densify 不检碰：逐点 + 邻段复核，拒绝「规划器认为可行但样本穿模」
	if (armReq.options.checkCollision && armReq.world)
	{
		for (std::size_t i = 0; i < out.jointTrajectoryRad.size(); ++i)
		{
			if (!detail::isStateValid(armReq, lim, out.jointTrajectoryRad[i]))
			{
				out.ok = false;
				out.errMsg = "path state in collision after densify (i=" + std::to_string(i) + ")";
				out.jointTrajectoryRad.clear();
				out.tcpPoses.clear();
				return false;
			}
			if (i + 1 < out.jointTrajectoryRad.size()
				&& !detail::isSegmentValid(armReq, lim, out.jointTrajectoryRad[i], out.jointTrajectoryRad[i + 1],
										   armReq.options.longestValidSegmentRad))
			{
				out.ok = false;
				out.errMsg = "path segment in collision after densify (i=" + std::to_string(i) + ")";
				out.jointTrajectoryRad.clear();
				out.tcpPoses.clear();
				return false;
			}
		}
	}
	detail::fillTcpPosesFromJoints(armReq, out);
	if (out.tcpPoses.size() != out.jointTrajectoryRad.size())
	{
		out.ok = false;
		out.errMsg = "TCP pose count mismatch after FK";
		return false;
	}
	detail::computePathMetrics(out);
	out.ok = true;
	return true;
}

} // namespace robot_path
