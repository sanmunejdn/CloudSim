/// @file CollisionValidity.cpp
/// @brief 关节状态/线段碰撞与限位

#include "CollisionValidity.h"

#include "Adapters.h"
#include "UrdfRobotLoader.h"

#include <algorithm>
#include <cmath>

namespace robot_path
{
namespace detail
{
namespace
{

/// CollisionWorld：Eigen 列主序 t@12..14；勿直接用 colMajorFromRigidTransform（OSG 底行序）
collision::Mat4 collisionMat4FromRigid(const engine::RigidTransform& rt)
{
	const Eigen::Isometry3d& iso = rt.isometry();
	collision::Mat4 out{};
	for (int r = 0; r < 3; ++r)
	{
		for (int c = 0; c < 3; ++c)
			out[static_cast<std::size_t>(c * 4 + r)] = iso.linear()(r, c);
	}
	out[12] = iso.translation().x();
	out[13] = iso.translation().y();
	out[14] = iso.translation().z();
	out[15] = 1.0;
	return out;
}

engine::RigidTransform rigidFromBackend(const BackendMat4& m)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
		cm[static_cast<std::size_t>(i)] = m.v[i];
	return engine::rigidTransformFromColMajor(cm);
}

osg::Matrixd osgFromBackend(const BackendMat4& m)
{
	return engine::osgMatrixFromRigidTransform(rigidFromBackend(m));
}

QVector<double> toQVector(const std::vector<double>& q)
{
	QVector<double> out;
	out.reserve(static_cast<int>(q.size()));
	for (double v : q)
		out.push_back(v);
	return out;
}

bool jointsNear(const std::vector<double>& a, const std::vector<double>& b, const double eps = 1e-9)
{
	if (a.size() != b.size())
		return false;
	for (std::size_t i = 0; i < a.size(); ++i)
	{
		if (std::abs(a[i] - b[i]) > eps)
			return false;
	}
	return true;
}

bool updateRobotPoses(const PlanRequest& req, const std::vector<double>& q)
{
	if (!req.world || req.linkBodies.isEmpty())
	{
		return false;
	}
	const bool nearStart = jointsNear(q, req.startJointRad);
	if (nearStart)
	{
		for (auto it = req.linkBodies.constBegin(); it != req.linkBodies.constEnd(); ++it)
		{
			const auto w0It = req.linkWorldAtStart.constFind(it.key());
			if (w0It == req.linkWorldAtStart.constEnd())
			{
				return false;
			}
			const osg::Matrixd W0 = osgFromBackend(*w0It);
			req.world->setWorldPose(it.value(), collisionMat4FromRigid(engine::rigidTransformFromOsg(W0)),
									"osg-start");
		}
		return true;
	}

	QHash<QString, osg::Matrixd> Tq;
	QString err;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(req.urdfPath, toQVector(q), Tq, &err, req.meshVerticesInLinkFrame))
	{
		return false;
	}

	const bool useBindPose = !req.fkMeshWorldT0.isEmpty() && !req.outerWorldAtBindByBackendId.isEmpty();
	const engine::RigidTransform T_world_base = rigidFromBackend(req.T_world_urdfBase);

	for (auto it = req.linkBodies.constBegin(); it != req.linkBodies.constEnd(); ++it)
	{
		const auto tqIt = Tq.constFind(it.key());
		if (tqIt == Tq.constEnd())
		{
			return false;
		}

		if (useBindPose)
		{
			const auto t0It = req.fkMeshWorldT0.constFind(it.key());
			const QString bid = QString::fromStdString(it.value().backendId);
			const auto m0It = req.outerWorldAtBindByBackendId.constFind(bid);
			// 缺绑定时不得跳过，否则沿用陈旧位姿 → 规划器假无碰、画面复验才撞上
			if (t0It == req.fkMeshWorldT0.constEnd() || m0It == req.outerWorldAtBindByBackendId.constEnd())
			{
				return false;
			}
			const osg::Matrixd Mworld =
				(*m0It) * osg::Matrixd::inverse(*t0It) * (*tqIt) * req.robotBasePlacementWorld;
			req.world->setWorldPose(it.value(), collisionMat4FromRigid(engine::rigidTransformFromOsg(Mworld)),
									"fk-bind");
			continue;
		}

		const engine::RigidTransform T_base_mesh = engine::rigidTransformFromOsg(*tqIt);
		const engine::RigidTransform T_world_mesh = T_world_base.composeScene(T_base_mesh);
		req.world->setWorldPose(it.value(), collisionMat4FromRigid(T_world_mesh), "fk");
	}
	return true;
}

} // namespace

bool loadJointLimits(const QString& urdfPath, JointLimits& out, std::string* errMsg)
{
	QStringList names;
	QVector<double> lower;
	QVector<double> upper;
	QString qerr;
	if (!UrdfRobotLoader::loadRevoluteJointMeta(urdfPath, names, lower, upper, &qerr))
	{
		if (errMsg)
			*errMsg = qerr.toStdString();
		return false;
	}
	out.lowerRad.assign(lower.begin(), lower.end());
	out.upperRad.assign(upper.begin(), upper.end());
	return !out.lowerRad.empty();
}

bool isWithinLimits(const std::vector<double>& q, const JointLimits& lim)
{
	if (q.size() != lim.lowerRad.size())
		return false;
	const double eps = 1e-6;
	for (std::size_t i = 0; i < q.size(); ++i)
	{
		if (q[i] < lim.lowerRad[i] - eps || q[i] > lim.upperRad[i] + eps)
			return false;
	}
	return true;
}

bool isStateValid(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& q)
{
	if (!isWithinLimits(q, lim))
		return false;
	if (!req.options.checkCollision || !req.world)
		return true;
	req.world->setSecurityMarginMm(req.options.securityMarginMm);
	if (!updateRobotPoses(req, q))
		return false;
	const collision::CollisionQueryResult hit = req.world->checkAll(4);
	return !hit.inCollision;
}

std::string describeStateInvalid(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& q)
{
	if (q.size() != lim.lowerRad.size())
	{
		return "joint dimension mismatch (q=" + std::to_string(q.size()) +
			   ", limits=" + std::to_string(lim.lowerRad.size()) + ")";
	}
	const double eps = 1e-6;
	for (std::size_t i = 0; i < q.size(); ++i)
	{
		if (q[i] < lim.lowerRad[i] - eps || q[i] > lim.upperRad[i] + eps)
			return "joint " + std::to_string(i) + " out of limits";
	}
	if (!req.options.checkCollision || !req.world)
		return {};
	req.world->setSecurityMarginMm(req.options.securityMarginMm);
	if (!updateRobotPoses(req, q))
		return "failed to update robot collision poses (FK/bind incomplete)";
	const collision::CollisionQueryResult hit = req.world->checkAll(4);
	if (!hit.inCollision)
		return {};
	std::string s = hit.summary.empty() ? "in collision" : hit.summary;
	if (!hit.contacts.empty())
	{
		const auto& c = hit.contacts.front();
		s += " @(" + std::to_string(c.pointMm[0]) + "," + std::to_string(c.pointMm[1]) + "," +
			 std::to_string(c.pointMm[2]) + ")";
	}
	return s;
}

bool isSegmentValid(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& a,
					const std::vector<double>& b, const double longestValidSegmentRad)
{
	if (a.size() != b.size())
		return false;
	double maxSpan = 0.0;
	for (std::size_t i = 0; i < a.size(); ++i)
		maxSpan = std::max(maxSpan, std::abs(b[i] - a[i]));
	const int steps = std::max(1, static_cast<int>(std::ceil(maxSpan / std::max(1e-6, longestValidSegmentRad))));
	for (int s = 0; s <= steps; ++s)
	{
		const double t = static_cast<double>(s) / static_cast<double>(steps);
		std::vector<double> q(a.size());
		for (std::size_t i = 0; i < a.size(); ++i)
			q[i] = a[i] + t * (b[i] - a[i]);
		if (!isStateValid(req, lim, q))
			return false;
	}
	return true;
}

} // namespace detail
} // namespace robot_path
