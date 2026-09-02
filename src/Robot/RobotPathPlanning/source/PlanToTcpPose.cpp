/// @file PlanToTcpPose.cpp
/// @brief planToTcpPose 入口：IK 目标 + 关节空间规划 + 双输出

#include "RobotPathPlanning.h"

#include "CollisionValidity.h"
#include "JointSpaceDijkstraPlanner.h"
#include "JointSpaceRrtPlanner.h"
#include "OmplJointSpacePlanner.h"
#include "PathPostProcess.h"
#include "TaskSpaceRrtPlanner.h"

#include "ToolKinematics.h"
#include "Adapters.h"
#include "UrdfIkSolverOptions.h"
#include "UrdfNumericalIk.h"
#include "UrdfRobotLoader.h"

#include <cmath>

#include <algorithm>
#include <string>

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

bool isAutoPlannerId(const std::string& id)
{
	return id.empty() || id == "Auto" || id == "auto";
}

std::string normalizePlanningSpace(const std::string& id)
{
	if (id == "Joint" || id == "joint")
		return "Joint";
	if (id == "Cartesian" || id == "cartesian")
		return "Cartesian";
	return "Auto";
}

std::string normalizeExplicitPlannerId(const std::string& id)
{
	if (id == "BIT*" || id == "BITstar")
		return "BITstar";
	if (id == "RRT*")
		return "RRTstar";
	if (id == "InformedRRT*")
		return "InformedRRTstar";
	if (id == "RRTConnect" || id == "RRTstar" || id == "InformedRRTstar" || id == "Dijkstra")
		return id;
	return "RRTConnect";
}

bool plannerRequiresOmpl(const std::string& id)
{
	return id == "BITstar" || id == "InformedRRTstar" || id == "RRTstar";
}

} // namespace

std::vector<std::string> supportedPlanningSpaces()
{
	return {"Auto", "Joint", "Cartesian"};
}

std::vector<std::string> supportedPlannerIds()
{
	std::vector<std::string> ids;
	ids.push_back("Auto");
#if defined(CLOUDSIM_HAS_OMPL)
	ids.push_back("BITstar");
	ids.push_back("InformedRRTstar");
	ids.push_back("RRTstar");
#endif
	ids.push_back("RRTConnect");
	ids.push_back("Dijkstra");
	return ids;
}

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
	if (!armReq.goalJointRad.empty())
	{
		goalQ = armReq.goalJointRad;
		if (goalQ.size() > lim.lowerRad.size())
			goalQ.resize(lim.lowerRad.size());
		if (goalQ.size() != lim.lowerRad.size())
		{
			out.errMsg = "goal joint count mismatch (got " + std::to_string(armReq.goalJointRad.size()) + ", urdf " +
						 std::to_string(lim.lowerRad.size()) + ")";
			return false;
		}
	}
	else if (!solveGoalJoints(armReq, goalQ, out.errMsg))
	{
		return false;
	}

	{
		double dq2 = 0.0;
		for (std::size_t i = 0; i < goalQ.size(); ++i)
		{
			const double d = goalQ[i] - armReq.startJointRad[i];
			dq2 += d * d;
		}
		// 起终点关节几乎重合时 Direct 会“成功”但 TCP 长为 0，拒绝以免假成功
		if (std::sqrt(dq2) < 1e-3)
		{
			out.errMsg = "start/goal joints nearly identical; re-teach start waypoint or pick different end";
			return false;
		}
	}

	// 采样规划前先诊断起/终点，避免起点碰场景时 OMPL 空转超时
	{
		const std::string startBad = detail::describeStateInvalid(armReq, lim, armReq.startJointRad);
		const std::string goalBad = detail::describeStateInvalid(armReq, lim, goalQ);
		if (!startBad.empty())
		{
			out.errMsg = "start state invalid: " + startBad;
			return false;
		}
		if (!goalBad.empty())
		{
			out.errMsg = "goal state invalid: " + goalBad;
			return false;
		}
	}

	auto densifyCollisionOk = [&](std::string& densifyErr) -> bool {
		const double stepRad = std::max(1e-6, armReq.options.longestValidSegmentRad);
		detail::densifyJointPath(out, stepRad);
		if (!armReq.options.checkCollision)
			return true;
		if (!armReq.world)
		{
			densifyErr = "collision check enabled but collision world is null";
			return false;
		}
		for (std::size_t i = 0; i < out.jointTrajectoryRad.size(); ++i)
		{
			if (!detail::isStateValid(armReq, lim, out.jointTrajectoryRad[i]))
			{
				densifyErr = "path state in collision after densify (i=" + std::to_string(i) + ")";
				return false;
			}
			if (i + 1 < out.jointTrajectoryRad.size()
				&& !detail::isSegmentValid(armReq, lim, out.jointTrajectoryRad[i], out.jointTrajectoryRad[i + 1],
										   stepRad))
			{
				densifyErr = "path segment in collision after densify (i=" + std::to_string(i) + ")";
				return false;
			}
		}
		return true;
	};

	auto snapGoalAndDensify = [&](std::string& err) -> bool {
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
						err = "path end != goal joints after plan";
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
		if (!densifyCollisionOk(err))
			return false;
		// 捷径缩短；若加密后穿模则退回捷径前稠密路径
		const auto backup = out.jointTrajectoryRad;
		const std::string nameBackup = out.plannerName;
		detail::shortcutJointPath(armReq, lim, out);
		std::string shortcutErr;
		if (!densifyCollisionOk(shortcutErr))
		{
			out.jointTrajectoryRad = backup;
			out.plannerName = nameBackup;
		}
		return true;
	};

	auto tryOmplOrRrt = [&](std::string& lastErr) -> bool {
		auto tryOne = [&](const char* plannerId, const double minTimeSec, const bool useOmpl) -> bool {
			PlanRequest tryReq = armReq;
			tryReq.options.plannerId = plannerId;
			tryReq.options.planningTimeSec = std::max(minTimeSec, armReq.options.planningTimeSec);
			bool planned = false;
#if defined(CLOUDSIM_HAS_OMPL)
			if (useOmpl)
				planned = detail::planJointSpaceOmpl(tryReq, lim, goalQ, out);
#endif
			if (!useOmpl)
				planned = detail::planJointSpaceRrt(tryReq, lim, goalQ, out);
			if (!planned)
			{
				lastErr = out.errMsg.empty() ? lastErr : out.errMsg;
				return false;
			}
			std::string densifyErr;
			if (!snapGoalAndDensify(densifyErr))
			{
				lastErr = densifyErr;
				out = PathResult{};
				return false;
			}
			return true;
		};

		const std::string requested = armReq.options.plannerId;
		if (!isAutoPlannerId(requested))
		{
			std::string selected = normalizeExplicitPlannerId(requested);
			if (selected == "Dijkstra")
			{
				PlanRequest tryReq = armReq;
				tryReq.options.plannerId = "Dijkstra";
				if (detail::planJointSpaceDijkstra(tryReq, lim, goalQ, out))
				{
					std::string densifyErr;
					if (snapGoalAndDensify(densifyErr))
						return true;
					lastErr = densifyErr;
					out = PathResult{};
				}
				else
				{
					lastErr = out.errMsg.empty() ? lastErr : out.errMsg;
				}
				if (out.errMsg.empty())
					out.errMsg = lastErr.empty() ? "planner Dijkstra failed" : lastErr;
				return false;
			}
#if !defined(CLOUDSIM_HAS_OMPL)
			if (plannerRequiresOmpl(selected))
			{
				selected = "RRTConnect";
				lastErr = "OMPL not available; using built-in RRTConnect for " + requested;
			}
#endif
			const bool useOmpl = [&] {
#if defined(CLOUDSIM_HAS_OMPL)
				return plannerRequiresOmpl(selected) || selected == "RRTConnect";
#else
				(void)selected;
				return false;
#endif
			}();
			const double minTime = selected == "RRTConnect" ? 5.0 : (selected == "RRTstar" ? 8.0 : 10.0);
			if (tryOne(selected.c_str(), minTime, useOmpl))
				return true;
			if (out.errMsg.empty())
				out.errMsg = lastErr.empty() ? ("planner " + selected + " failed") : lastErr;
			return false;
		}

#if defined(CLOUDSIM_HAS_OMPL)
		if (tryOne("BITstar", 10.0, true))
			return true;
		if (tryOne("InformedRRTstar", 10.0, true))
			return true;
		if (tryOne("RRTstar", 8.0, true))
			return true;
		if (tryOne("RRTConnect", 5.0, true))
			return true;
#endif
		if (tryOne("RRTConnect", 5.0, false))
			return true;
		if (out.errMsg.empty())
			out.errMsg = lastErr.empty() ? "motion planning failed" : lastErr;
		return false;
	};

	auto finalizePath = [&]() -> bool {
		detail::fillTcpPosesFromJoints(armReq, out);
		if (out.tcpPoses.size() != out.jointTrajectoryRad.size())
		{
			out.ok = false;
			out.errMsg = "TCP pose count mismatch after FK";
			return false;
		}
		detail::computePathMetrics(out);
		constexpr double kMinUsefulTcpMm = 1.0;
		constexpr double kMinUsefulJointRad = 0.02;
		if (out.pathLengthTcpMm < kMinUsefulTcpMm && out.pathLengthRad < kMinUsefulJointRad)
		{
			out.ok = false;
			out.errMsg = "planned path too short (TCP=" + std::to_string(out.pathLengthTcpMm) +
						 "mm); start/goal poses nearly coincident";
			out.jointTrajectoryRad.clear();
			out.tcpPoses.clear();
			return false;
		}
		out.ok = true;
		return true;
	};

	auto tryTaskSpaceRrt = [&]() -> bool {
		if (!detail::planTaskSpaceRrt(armReq, lim, goalQ, out))
			return false;
		std::string densifyErr;
		if (snapGoalAndDensify(densifyErr))
			return true;
		// densify 关节插值可能穿模；树边已是 TCP-IK 小步，粗路径段检通过则仍可用
		if (!out.jointTrajectoryRad.empty())
		{
			bool coarseOk = true;
			for (std::size_t i = 0; i + 1 < out.jointTrajectoryRad.size(); ++i)
			{
				if (!detail::isStateValid(armReq, lim, out.jointTrajectoryRad[i])
					|| !detail::isSegmentValid(armReq, lim, out.jointTrajectoryRad[i],
											   out.jointTrajectoryRad[i + 1],
											   armReq.options.longestValidSegmentRad))
				{
					coarseOk = false;
					break;
				}
			}
			if (coarseOk && detail::isStateValid(armReq, lim, out.jointTrajectoryRad.back()))
			{
				out.ok = true;
				return true;
			}
		}
		out = PathResult{};
		out.errMsg = densifyErr.empty() ? "TaskSpaceRRT post-process failed" : densifyErr;
		return false;
	};

	const std::string planningSpace = normalizePlanningSpace(armReq.options.planningSpace);
	if (planningSpace == "Cartesian")
	{
		if (!tryTaskSpaceRrt())
			return false;
		return finalizePath();
	}
	if (planningSpace == "Auto")
	{
		if (tryTaskSpaceRrt())
			return finalizePath();
		out = PathResult{};
	}

	bool usedDirect = false;
	// 无障碍/直达可行时：关节直线最短且可复现
	if (armReq.options.allowDirectJointLerp
		&& detail::isSegmentValid(armReq, lim, armReq.startJointRad, goalQ, armReq.options.longestValidSegmentRad))
	{
		out.jointTrajectoryRad = {armReq.startJointRad, goalQ};
		out.plannerName = "Direct";
		out.ok = true;
		usedDirect = true;
	}
	else
	{
		std::string lastErr;
		if (!tryOmplOrRrt(lastErr))
			return false;
	}

	if (usedDirect)
	{
		std::string densifyErr;
		if (!snapGoalAndDensify(densifyErr))
		{
			// Direct 粗检漏过中段穿模：改走 OMPL 绕障，勿整段直接失败
			out = PathResult{};
			std::string lastErr;
			if (!tryOmplOrRrt(lastErr))
			{
				out.ok = false;
				out.errMsg = densifyErr + "; ompl fallback failed: " + (lastErr.empty() ? out.errMsg : lastErr);
				out.jointTrajectoryRad.clear();
				out.tcpPoses.clear();
				return false;
			}
		}
	}

	return finalizePath();
}

} // namespace robot_path
