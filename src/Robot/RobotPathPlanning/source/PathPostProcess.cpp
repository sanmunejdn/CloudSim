/// @file PathPostProcess.cpp
/// @brief 关节路径 → TCP 位姿列

#include "PathPostProcess.h"

#include "Adapters.h"
#include "CollisionValidity.h"
#include "ToolKinematics.h"
#include "UrdfRobotLoader.h"

#include <cmath>

namespace robot_path
{
namespace detail
{
namespace
{

engine::RigidTransform rigidFromBackend(const BackendMat4& m)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
		cm[static_cast<size_t>(i)] = m.v[i];
	return engine::rigidTransformFromColMajor(cm);
}

QVector<double> toQVector(const std::vector<double>& q)
{
	QVector<double> out;
	out.reserve(static_cast<int>(q.size()));
	for (double v : q)
		out.push_back(v);
	return out;
}

} // namespace

void fillTcpPosesFromJoints(const PlanRequest& req, PathResult& io)
{
	io.tcpPoses.clear();
	io.tcpPoses.reserve(io.jointTrajectoryRad.size());
	const engine::RigidTransform T_tool = rigidFromBackend(req.T_flange_tool);
	const engine::RigidTransform T_world_base = rigidFromBackend(req.T_world_urdfBase);

	for (const auto& q : io.jointTrajectoryRad)
	{
		QHash<QString, engine::RigidTransform> linkWorld;
		QString err;
		if (!UrdfRobotLoader::computeLinkWorldRigidTransforms(req.urdfPath, toQVector(q), linkWorld, &err))
			continue;
		const auto it = linkWorld.constFind(req.flangeLinkName);
		if (it == linkWorld.constEnd())
			continue;
		const engine::RigidTransform T_base_tool = engine::toolOriginFromFlange(*it, T_tool);
		// Eigen 列向量链：勿 composeScene（与 ToolKinematics 约定一致）
		const engine::RigidTransform T_world_tool = T_world_base.composeColumn(T_base_tool);
		TcpPose pose{};
		T_world_tool.translationMm(pose.transMm[0], pose.transMm[1], pose.transMm[2]);
		const Eigen::Quaterniond quat = T_world_tool.rotation();
		pose.quatXyzw[0] = quat.x();
		pose.quatXyzw[1] = quat.y();
		pose.quatXyzw[2] = quat.z();
		pose.quatXyzw[3] = quat.w();
		io.tcpPoses.push_back(pose);
	}
}

void densifyJointPath(PathResult& io, const double maxStepRad)
{
	auto& path = io.jointTrajectoryRad;
	if (path.size() < 2 || maxStepRad <= 1e-9)
		return;
	std::vector<std::vector<double>> out;
	out.reserve(path.size() * 4);
	out.push_back(path.front());
	for (std::size_t i = 1; i < path.size(); ++i)
	{
		const auto& a = path[i - 1];
		const auto& b = path[i];
		double span = 0.0;
		for (std::size_t j = 0; j < a.size(); ++j)
		{
			const double d = b[j] - a[j];
			span += d * d;
		}
		span = std::sqrt(span);
		const int steps = std::max(1, static_cast<int>(std::ceil(span / maxStepRad)));
		for (int s = 1; s <= steps; ++s)
		{
			const double t = static_cast<double>(s) / static_cast<double>(steps);
			std::vector<double> q(a.size());
			for (std::size_t j = 0; j < a.size(); ++j)
				q[j] = a[j] + t * (b[j] - a[j]);
			out.push_back(std::move(q));
		}
	}
	path.swap(out);
}

void shortcutJointPath(const PlanRequest& req, const JointLimits& lim, PathResult& io)
{
	auto& path = io.jointTrajectoryRad;
	if (path.size() < 3)
		return;
	const double seg = req.options.longestValidSegmentRad;
	std::vector<std::vector<double>> out;
	out.reserve(path.size());
	out.push_back(path.front());
	std::size_t i = 0;
	while (i + 1 < path.size())
	{
		std::size_t best = i + 1;
		for (std::size_t j = path.size() - 1; j > i + 1; --j)
		{
			if (isSegmentValid(req, lim, path[i], path[j], seg))
			{
				best = j;
				break;
			}
		}
		out.push_back(path[best]);
		i = best;
	}
	path.swap(out);
}

void computePathMetrics(PathResult& io)
{
	io.pathLengthRad = 0.0;
	io.pathLengthTcpMm = 0.0;
	for (std::size_t i = 1; i < io.jointTrajectoryRad.size(); ++i)
	{
		const auto& a = io.jointTrajectoryRad[i - 1];
		const auto& b = io.jointTrajectoryRad[i];
		double s = 0.0;
		for (std::size_t j = 0; j < a.size(); ++j)
		{
			const double d = b[j] - a[j];
			s += d * d;
		}
		io.pathLengthRad += std::sqrt(s);
	}
	for (std::size_t i = 1; i < io.tcpPoses.size(); ++i)
	{
		const TcpPose& a = io.tcpPoses[i - 1];
		const TcpPose& b = io.tcpPoses[i];
		const double dx = b.transMm[0] - a.transMm[0];
		const double dy = b.transMm[1] - a.transMm[1];
		const double dz = b.transMm[2] - a.transMm[2];
		io.pathLengthTcpMm += std::sqrt(dx * dx + dy * dy + dz * dz);
	}
}

} // namespace detail
} // namespace robot_path
