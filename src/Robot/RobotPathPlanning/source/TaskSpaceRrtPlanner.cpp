/// @file TaskSpaceRrtPlanner.cpp
/// @brief SE(3) 任务空间 RRTConnect + IK

#include "TaskSpaceRrtPlanner.h"

#include "Adapters.h"
#include "ToolKinematics.h"
#include "UrdfIkSolverOptions.h"
#include "UrdfNumericalIk.h"
#include "UrdfRobotLoader.h"

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <random>
#include <vector>

namespace robot_path
{
namespace detail
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

void tcpPoseFromRigid(const engine::RigidTransform& T, TcpPose& out)
{
	T.translationMm(out.transMm[0], out.transMm[1], out.transMm[2]);
	const Eigen::Quaterniond q = T.rotation().normalized();
	out.quatXyzw[0] = q.x();
	out.quatXyzw[1] = q.y();
	out.quatXyzw[2] = q.z();
	out.quatXyzw[3] = q.w();
}

bool tcpToolInBaseFromJoints(const PlanRequest& req, const std::vector<double>& q, TcpPose& out)
{
	QVector<double> qv;
	qv.reserve(static_cast<int>(q.size()));
	for (double v : q)
		qv.push_back(v);
	QHash<QString, engine::RigidTransform> linkWorld;
	QString err;
	if (!UrdfRobotLoader::computeLinkWorldRigidTransforms(req.urdfPath, qv, linkWorld, &err))
		return false;
	const auto it = linkWorld.constFind(req.flangeLinkName);
	if (it == linkWorld.constEnd())
		return false;
	const engine::RigidTransform T_tool = rigidFromBackend(req.T_flange_tool);
	const engine::RigidTransform T_base_tool = engine::toolOriginFromFlange(*it, T_tool);
	tcpPoseFromRigid(T_base_tool, out);
	return true;
}

bool ikToolInBase(const PlanRequest& req, const TcpPose& toolInBase, const std::vector<double>& seedQ,
				  std::vector<double>& outQ)
{
	const engine::RigidTransform T_base_target = rigidFromTcpPose(toolInBase);
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
	outQ = UrdfRobotLoader::solveArmPoseDampedLeastSquares(req.urdfPath, req.flangeLinkName, target, seedQ, opts,
														   &ikFail);
	return !outQ.empty();
}

double poseDistanceMm(const TcpPose& a, const TcpPose& b, const double rotWeightMmPerRad)
{
	const double dx = a.transMm[0] - b.transMm[0];
	const double dy = a.transMm[1] - b.transMm[1];
	const double dz = a.transMm[2] - b.transMm[2];
	const double trans = std::sqrt(dx * dx + dy * dy + dz * dz);
	Eigen::Quaterniond qa(a.quatXyzw[3], a.quatXyzw[0], a.quatXyzw[1], a.quatXyzw[2]);
	Eigen::Quaterniond qb(b.quatXyzw[3], b.quatXyzw[0], b.quatXyzw[1], b.quatXyzw[2]);
	if (qa.coeffs().hasNaN() || qb.coeffs().hasNaN())
		return trans + rotWeightMmPerRad;
	const double angle = qa.normalized().angularDistance(qb.normalized());
	return trans + rotWeightMmPerRad * angle;
}

TcpPose interpolateTcp(const TcpPose& a, const TcpPose& b, const double u)
{
	TcpPose out{};
	const double t = std::clamp(u, 0.0, 1.0);
	out.transMm[0] = a.transMm[0] * (1.0 - t) + b.transMm[0] * t;
	out.transMm[1] = a.transMm[1] * (1.0 - t) + b.transMm[1] * t;
	out.transMm[2] = a.transMm[2] * (1.0 - t) + b.transMm[2] * t;
	Eigen::Quaterniond qa(a.quatXyzw[3], a.quatXyzw[0], a.quatXyzw[1], a.quatXyzw[2]);
	Eigen::Quaterniond qb(b.quatXyzw[3], b.quatXyzw[0], b.quatXyzw[1], b.quatXyzw[2]);
	Eigen::Quaterniond q = qa.normalized().slerp(t, qb.normalized());
	if (q.coeffs().hasNaN())
		q = qb.normalized();
	out.quatXyzw[0] = q.x();
	out.quatXyzw[1] = q.y();
	out.quatXyzw[2] = q.z();
	out.quatXyzw[3] = q.w();
	return out;
}

TcpPose steerTcp(const TcpPose& from, const TcpPose& to, const double maxStepMm, const double maxStepRad)
{
	const double dx = to.transMm[0] - from.transMm[0];
	const double dy = to.transMm[1] - from.transMm[1];
	const double dz = to.transMm[2] - from.transMm[2];
	const double trans = std::sqrt(dx * dx + dy * dy + dz * dz);
	Eigen::Quaterniond qa(from.quatXyzw[3], from.quatXyzw[0], from.quatXyzw[1], from.quatXyzw[2]);
	Eigen::Quaterniond qb(to.quatXyzw[3], to.quatXyzw[0], to.quatXyzw[1], to.quatXyzw[2]);
	const double angle = qa.normalized().angularDistance(qb.normalized());
	double u = 1.0;
	if (trans > 1e-6)
		u = std::min(u, maxStepMm / trans);
	if (angle > 1e-6)
		u = std::min(u, maxStepRad / angle);
	if (u >= 1.0 - 1e-9)
		return to;
	return interpolateTcp(from, to, u);
}

struct TaskNode
{
	TcpPose pose{};
	std::vector<double> q;
	int parent = -1;
};

/// TCP 段逐点 IK；写出不含起点、含终点的关节链（供 densify 不再关节直线穿模）
bool connectPoses(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& qFrom,
				  const TcpPose& poseFrom, const TcpPose& poseTo, const double maxStepMm, const double maxStepRad,
				  std::vector<TcpPose>& outPoses, std::vector<std::vector<double>>& outQs)
{
	outPoses.clear();
	outQs.clear();
	const double dx = poseTo.transMm[0] - poseFrom.transMm[0];
	const double dy = poseTo.transMm[1] - poseFrom.transMm[1];
	const double dz = poseTo.transMm[2] - poseFrom.transMm[2];
	const double trans = std::sqrt(dx * dx + dy * dy + dz * dz);
	Eigen::Quaterniond qa(poseFrom.quatXyzw[3], poseFrom.quatXyzw[0], poseFrom.quatXyzw[1], poseFrom.quatXyzw[2]);
	Eigen::Quaterniond qb(poseTo.quatXyzw[3], poseTo.quatXyzw[0], poseTo.quatXyzw[1], poseTo.quatXyzw[2]);
	const double angle = qa.normalized().angularDistance(qb.normalized());
	const int stepsT = std::max(1, static_cast<int>(std::ceil(trans / std::max(1.0, maxStepMm))));
	const int stepsR = std::max(1, static_cast<int>(std::ceil(angle / std::max(0.01, maxStepRad))));
	const int steps = std::clamp(std::max(stepsT, stepsR), 1, 24);

	std::vector<double> prevQ = qFrom;
	for (int i = 1; i <= steps; ++i)
	{
		const double u = static_cast<double>(i) / static_cast<double>(steps);
		const TcpPose mid = interpolateTcp(poseFrom, poseTo, u);
		std::vector<double> midQ;
		if (!ikToolInBase(req, mid, prevQ, midQ))
			return false;
		if (!isWithinLimits(midQ, lim))
			return false;
		if (!isStateValid(req, lim, midQ))
			return false;
		if (!isSegmentValid(req, lim, prevQ, midQ, req.options.longestValidSegmentRad))
			return false;
		outPoses.push_back(mid);
		outQs.push_back(midQ);
		prevQ = std::move(midQ);
	}
	return !outQs.empty();
}

int nearestNode(const std::vector<TaskNode>& nodes, const TcpPose& pose, const double rotWeightMmPerRad)
{
	int best = 0;
	double bestD = std::numeric_limits<double>::infinity();
	for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
	{
		const double d = poseDistanceMm(nodes[static_cast<size_t>(i)].pose, pose, rotWeightMmPerRad);
		if (d < bestD)
		{
			bestD = d;
			best = i;
		}
	}
	return best;
}

bool extractJointPath(const std::vector<TaskNode>& nodes, int goalIdx, std::vector<std::vector<double>>& path)
{
	path.clear();
	if (goalIdx < 0 || goalIdx >= static_cast<int>(nodes.size()))
		return false;
	int cur = goalIdx;
	while (cur >= 0)
	{
		path.push_back(nodes[static_cast<size_t>(cur)].q);
		cur = nodes[static_cast<size_t>(cur)].parent;
	}
	std::reverse(path.begin(), path.end());
	return path.size() >= 2;
}

int appendChain(std::vector<TaskNode>& tree, int parentIdx, const std::vector<TcpPose>& poses,
				const std::vector<std::vector<double>>& qs)
{
	int cur = parentIdx;
	for (std::size_t i = 0; i < qs.size(); ++i)
	{
		TaskNode n;
		n.pose = poses[i];
		n.q = qs[i];
		n.parent = cur;
		tree.push_back(std::move(n));
		cur = static_cast<int>(tree.size()) - 1;
	}
	return cur;
}

TcpPose sampleRandomPose(const TcpPose& startPose, const TcpPose& goalPose, std::mt19937& rng, const double marginMm)
{
	std::uniform_real_distribution<double> u01(0.0, 1.0);
	const double minX = std::min(startPose.transMm[0], goalPose.transMm[0]) - marginMm;
	const double maxX = std::max(startPose.transMm[0], goalPose.transMm[0]) + marginMm;
	const double minY = std::min(startPose.transMm[1], goalPose.transMm[1]) - marginMm;
	const double maxY = std::max(startPose.transMm[1], goalPose.transMm[1]) + marginMm;
	const double minZ = std::min(startPose.transMm[2], goalPose.transMm[2]) - marginMm;
	const double maxZ = std::max(startPose.transMm[2], goalPose.transMm[2]) + marginMm;
	std::uniform_real_distribution<double> ux(minX, maxX);
	std::uniform_real_distribution<double> uy(minY, maxY);
	std::uniform_real_distribution<double> uz(minZ, maxZ);

	TcpPose out{};
	out.transMm[0] = ux(rng);
	out.transMm[1] = uy(rng);
	out.transMm[2] = uz(rng);

	const TcpPose orient = interpolateTcp(startPose, goalPose, u01(rng));
	std::uniform_real_distribution<double> ang(-0.25, 0.25);
	Eigen::Quaterniond qBase(orient.quatXyzw[3], orient.quatXyzw[0], orient.quatXyzw[1], orient.quatXyzw[2]);
	const Eigen::Quaterniond qPert =
		(Eigen::Quaterniond(Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitZ()))
		 * Eigen::Quaterniond(Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitY()))
		 * Eigen::Quaterniond(Eigen::AngleAxisd(ang(rng), Eigen::Vector3d::UnitX())))
			.normalized();
	const Eigen::Quaterniond q = (qPert * qBase).normalized();
	out.quatXyzw[0] = q.x();
	out.quatXyzw[1] = q.y();
	out.quatXyzw[2] = q.z();
	out.quatXyzw[3] = q.w();
	return out;
}

bool tryCartesianLin(const PlanRequest& req, const JointLimits& lim, const TcpPose& startPose, const TcpPose& goalPose,
					 const std::vector<double>& startQ, const std::vector<double>& goalQ, PathResult& out)
{
	const double maxStepMm = std::max(5.0, req.options.taskSpaceMaxStepMm);
	const double maxStepRad = std::max(0.05, req.options.taskSpaceMaxStepRad);
	std::vector<TcpPose> poses;
	std::vector<std::vector<double>> qs;
	if (!connectPoses(req, lim, startQ, startPose, goalPose, maxStepMm, maxStepRad, poses, qs))
		return false;
	out.jointTrajectoryRad.clear();
	out.jointTrajectoryRad.push_back(startQ);
	for (auto& q : qs)
		out.jointTrajectoryRad.push_back(std::move(q));
	// 对齐终点关节，避免 IK 分支与 goalQ 差一截
	if (!out.jointTrajectoryRad.empty() && out.jointTrajectoryRad.back().size() == goalQ.size())
	{
		if (isSegmentValid(req, lim, out.jointTrajectoryRad.back(), goalQ, req.options.longestValidSegmentRad))
			out.jointTrajectoryRad.back() = goalQ;
	}
	out.ok = true;
	out.plannerName = "CartesianLIN";
	return true;
}

bool mergeStartGoalPaths(const std::vector<std::vector<double>>& pathStart,
						 const std::vector<std::vector<double>>& bridgeStartToGoal,
						 const std::vector<std::vector<double>>& pathGoalToRoot, PathResult& out)
{
	// pathGoalToRoot：goal→…→nearGoal；反转为 nearGoal→…→goal
	out.jointTrajectoryRad = pathStart;
	auto almostSame = [](const std::vector<double>& a, const std::vector<double>& b) {
		if (a.size() != b.size())
			return false;
		double dq = 0.0;
		for (std::size_t j = 0; j < a.size(); ++j)
		{
			const double d = a[j] - b[j];
			dq += d * d;
		}
		return std::sqrt(dq) < 1e-5;
	};
	for (const auto& q : bridgeStartToGoal)
	{
		if (!out.jointTrajectoryRad.empty() && almostSame(out.jointTrajectoryRad.back(), q))
			continue;
		out.jointTrajectoryRad.push_back(q);
	}
	for (std::size_t i = pathGoalToRoot.size(); i > 0; --i)
	{
		const auto& q = pathGoalToRoot[i - 1];
		if (!out.jointTrajectoryRad.empty() && almostSame(out.jointTrajectoryRad.back(), q))
			continue;
		out.jointTrajectoryRad.push_back(q);
	}
	out.ok = out.jointTrajectoryRad.size() >= 2;
	out.plannerName = "TaskSpaceRRT";
	return out.ok;
}

} // namespace

bool planTaskSpaceRrt(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& goalQ,
					  PathResult& out)
{
	out = PathResult{};
	const double maxStepMm = std::max(5.0, req.options.taskSpaceMaxStepMm);
	const double maxStepRad = std::max(0.05, req.options.taskSpaceMaxStepRad);
	const double goalBias = std::clamp(std::max(0.25, req.options.taskSpaceGoalBias), 0.0, 0.85);
	const double marginMm = std::max(150.0, req.options.taskSpaceSampleBoxMarginMm);
	constexpr double kRotWeightMmPerRad = 50.0;
	constexpr double kReachMm = 12.0;

	TcpPose startPose{};
	TcpPose goalPose{};
	if (!tcpToolInBaseFromJoints(req, req.startJointRad, startPose))
	{
		out.errMsg = "FK failed for start joints";
		return false;
	}
	if (!tcpToolInBaseFromJoints(req, goalQ, goalPose))
	{
		out.errMsg = "FK failed for goal joints";
		return false;
	}

	if (!isStateValid(req, lim, req.startJointRad))
	{
		out.errMsg = "start state invalid: " + describeStateInvalid(req, lim, req.startJointRad);
		return false;
	}
	if (!isStateValid(req, lim, goalQ))
	{
		out.errMsg = "goal state invalid: " + describeStateInvalid(req, lim, goalQ);
		return false;
	}

	if (tryCartesianLin(req, lim, startPose, goalPose, req.startJointRad, goalQ, out))
		return true;

	const auto t0 = std::chrono::steady_clock::now();
	const double budgetSec = std::max(1.0, req.options.planningTimeSec);
	std::mt19937 rng(req.options.rngSeed ^ 0xC0FFEEu);

	std::vector<TaskNode> treeStart;
	std::vector<TaskNode> treeGoal;
	treeStart.push_back(TaskNode{startPose, req.startJointRad, -1});
	treeGoal.push_back(TaskNode{goalPose, goalQ, -1});

	bool growStart = true;
	while (true)
	{
		const auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration<double>(now - t0).count() > budgetSec)
			break;

		std::vector<TaskNode>& ta = growStart ? treeStart : treeGoal;
		std::vector<TaskNode>& tb = growStart ? treeGoal : treeStart;
		const TcpPose& sampleGoal = growStart ? goalPose : startPose;

		std::uniform_real_distribution<double> u01(0.0, 1.0);
		const TcpPose rnd =
			(u01(rng) < goalBias) ? sampleGoal : sampleRandomPose(startPose, goalPose, rng, marginMm);

		const int nn = nearestNode(ta, rnd, kRotWeightMmPerRad);
		const TcpPose steered = steerTcp(ta[static_cast<size_t>(nn)].pose, rnd, maxStepMm, maxStepRad);
		std::vector<TcpPose> chainPoses;
		std::vector<std::vector<double>> chainQs;
		if (!connectPoses(req, lim, ta[static_cast<size_t>(nn)].q, ta[static_cast<size_t>(nn)].pose, steered,
						  maxStepMm, maxStepRad, chainPoses, chainQs))
		{
			growStart = !growStart;
			continue;
		}
		int newIdx = appendChain(ta, nn, chainPoses, chainQs);

		bool connected = false;
		int idxStart = -1;
		int idxGoal = -1;
		std::vector<std::vector<double>> bridgeStartToGoal;

		for (int grow = 0; grow < 64; ++grow)
		{
			const int nearB = nearestNode(tb, ta[static_cast<size_t>(newIdx)].pose, kRotWeightMmPerRad);
			const double dist = poseDistanceMm(ta[static_cast<size_t>(newIdx)].pose,
											   tb[static_cast<size_t>(nearB)].pose, kRotWeightMmPerRad);

			auto tryBridge = [&](int startIdx, int goalIdx) -> bool {
				std::vector<TcpPose> bridgePoses;
				std::vector<std::vector<double>> bridgeQs;
				if (!connectPoses(req, lim, treeStart[static_cast<size_t>(startIdx)].q,
								  treeStart[static_cast<size_t>(startIdx)].pose,
								  treeGoal[static_cast<size_t>(goalIdx)].pose, maxStepMm, maxStepRad, bridgePoses,
								  bridgeQs))
					return false;
				idxStart = startIdx;
				idxGoal = goalIdx;
				bridgeStartToGoal = std::move(bridgeQs);
				return true;
			};

			if (dist < kReachMm)
			{
				if (growStart)
					connected = tryBridge(newIdx, nearB);
				else
					connected = tryBridge(nearB, newIdx);
				break;
			}

			const TcpPose toward = steerTcp(ta[static_cast<size_t>(newIdx)].pose,
											tb[static_cast<size_t>(nearB)].pose, maxStepMm, maxStepRad);
			std::vector<TcpPose> morePoses;
			std::vector<std::vector<double>> moreQs;
			if (!connectPoses(req, lim, ta[static_cast<size_t>(newIdx)].q, ta[static_cast<size_t>(newIdx)].pose,
							  toward, maxStepMm, maxStepRad, morePoses, moreQs))
				break;
			newIdx = appendChain(ta, newIdx, morePoses, moreQs);

			const int nearB2 = nearestNode(tb, ta[static_cast<size_t>(newIdx)].pose, kRotWeightMmPerRad);
			if (poseDistanceMm(ta[static_cast<size_t>(newIdx)].pose, tb[static_cast<size_t>(nearB2)].pose,
							   kRotWeightMmPerRad)
				< kReachMm)
			{
				if (growStart)
					connected = tryBridge(newIdx, nearB2);
				else
					connected = tryBridge(nearB2, newIdx);
				break;
			}
		}

		if (connected && idxStart >= 0 && idxGoal >= 0)
		{
			std::vector<std::vector<double>> pathStart;
			std::vector<std::vector<double>> pathGoal;
			if (extractJointPath(treeStart, idxStart, pathStart) && extractJointPath(treeGoal, idxGoal, pathGoal)
				&& mergeStartGoalPaths(pathStart, bridgeStartToGoal, pathGoal, out))
			{
				if (!out.jointTrajectoryRad.empty() && out.jointTrajectoryRad.back().size() == goalQ.size())
				{
					if (isSegmentValid(req, lim, out.jointTrajectoryRad.back(), goalQ,
									   req.options.longestValidSegmentRad))
						out.jointTrajectoryRad.back() = goalQ;
					else
						out.jointTrajectoryRad.push_back(goalQ);
				}
				return true;
			}
		}
		growStart = !growStart;
	}

	out.errMsg = "TaskSpaceRRT: no path within time budget";
	return false;
}

} // namespace detail
} // namespace robot_path
