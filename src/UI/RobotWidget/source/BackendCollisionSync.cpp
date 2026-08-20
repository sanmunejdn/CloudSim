/// @file BackendCollisionSync.cpp
/// @brief 后端几何 → CollisionWorld；轨迹抽样校验

#include "BackendCollisionSync.h"

#include "BackendDataManager.h"
#include "BrepBackendData.h"
#include "CoreTypes.h"
#include "IRobotDocumentHost.h"
#include "IRobotOsgViewHost.h"
#include "MeshBackendData.h"
#include "MeshDiscretize.h"
#include "UrdfRobotLoader.h"

#include <Adapters.h>

#include <algorithm>
#include <cmath>
#include <unordered_set>

#include <osg/Matrixd>

namespace BackendCollisionSync
{
namespace
{
/// CollisionWorld 用 Eigen 列主序（t 在 12..14）；BackendMat4/colMajorFromRigidTransform 是 OSG 底行序（t 在 3/7/11）
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

collision::Mat4 collisionMat4FromBackend(const BackendMat4& m)
{
	engine::ColMajorMat4 cm{};
	for (int i = 0; i < 16; ++i)
		cm[static_cast<std::size_t>(i)] = m.v[i];
	return collisionMat4FromRigid(engine::rigidTransformFromColMajor(cm));
}

/// OsgWidget::getBackendRootWorldMatrix 的 Mat4 是 OSG 元素按 c*4+r 打包，不是 Backend/碰撞列主序
osg::Matrixd osgMatrixFromOsgPackedCoreMat4(const cloudsim::core::Mat4& packed)
{
	osg::Matrixd m;
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
			m(r, c) = packed[static_cast<std::size_t>(c * 4 + r)];
	}
	return m;
}

/// Gizmo/父子挂载只改 OSG 时，backend.worldMatrix 可能仍是局部或导入初值
struct ResolvedBodyWorld
{
	collision::Mat4 W{};
	const char* poseSource = "backend";
};

ResolvedBodyWorld resolveBodyWorld(IRobotOsgViewHost* osg, const std::shared_ptr<BackendDataBase>& data)
{
	ResolvedBodyWorld out;
	if (osg && data)
	{
		cloudsim::core::Mat4 packed{};
		if (osg->getBackendRootWorldMatrix(data->id(), packed))
		{
			const osg::Matrixd om = osgMatrixFromOsgPackedCoreMat4(packed);
			out.W = collisionMat4FromRigid(engine::rigidTransformFromOsg(om));
			out.poseSource = "osg";
			return out;
		}
	}
	if (!data)
	{
		out.W = collisionMat4FromRigid(engine::RigidTransform::identity());
		out.poseSource = "identity";
		return out;
	}
	// BackendMat4 为 OSG 底行序，须转成 CollisionWorld 的 Eigen 列主序
	out.W = collisionMat4FromBackend(data->worldMatrix());
	out.poseSource = "backend";
	return out;
}

collision::CollisionBodyId makeLinkBody(const QString& backendId, const QString& linkName)
{
	collision::CollisionBodyId id;
	id.kind = "robotLink";
	id.backendId = backendId.toStdString();
	id.linkName = linkName.toStdString();
	return id;
}

collision::CollisionBodyId makeSceneBody(const QString& backendId)
{
	collision::CollisionBodyId id;
	id.kind = "scene";
	id.backendId = backendId.toStdString();
	return id;
}

bool discretizeBrepSoup(const BrepBackendData& brep, std::vector<float>& soup)
{
	geoalgo::MeshDiscretizeParams params;
	params.quality = geoalgo::MeshQualityPreset::Coarse;
	geoalgo::MeshDiscretizeReport report;
	std::string err;
	return geoalgo::discretizeShapeHandleToMesh(brep.shapeRef(), params, soup, report, &err) && soup.size() >= 9;
}

void upsertFromBackend(collision::CollisionWorld& world, const collision::CollisionBodyId& id,
					   const std::shared_ptr<BackendDataBase>& data, IRobotOsgViewHost* osg,
					   std::vector<collision::CollisionBodyId>* outBodies)
{
	if (!data || !data->hasGeometry())
		return;
	const ResolvedBodyWorld resolved = resolveBodyWorld(osg, data);
	if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
	{
		const auto& soup = mesh->triangleSoup();
		if (soup.size() >= 9)
		{
			world.upsertMeshBody(id, soup.data(), soup.size(), resolved.W, resolved.poseSource);
			if (outBodies)
				outBodies->push_back(id);
		}
		return;
	}
	if (auto brep = std::dynamic_pointer_cast<BrepBackendData>(data))
	{
		std::vector<float> soup;
		if (discretizeBrepSoup(*brep, soup))
		{
			world.upsertMeshBody(id, soup.data(), soup.size(), resolved.W, resolved.poseSource);
			if (outBodies)
				outBodies->push_back(id);
		}
	}
}

/// 名单非空时：仅白↔黑互检；同名单与未入名单对一律排除
void applyWhiteBlackListFilter(collision::CollisionWorld& world, const RobotCollision::Settings& settings,
							   const std::vector<collision::CollisionBodyId>& bodies)
{
	if (settings.whiteListBackendIds.empty() && settings.blackListBackendIds.empty())
		return;

	std::unordered_set<std::string> white(settings.whiteListBackendIds.begin(), settings.whiteListBackendIds.end());
	std::unordered_set<std::string> black(settings.blackListBackendIds.begin(), settings.blackListBackendIds.end());

	auto listOf = [&](const collision::CollisionBodyId& id) -> int {
		if (white.count(id.backendId) > 0)
			return 1;
		if (black.count(id.backendId) > 0)
			return 2;
		return 0;
	};

	for (std::size_t i = 0; i < bodies.size(); ++i)
	{
		for (std::size_t j = i + 1; j < bodies.size(); ++j)
		{
			const int a = listOf(bodies[i]);
			const int b = listOf(bodies[j]);
			const bool cross = (a == 1 && b == 2) || (a == 2 && b == 1);
			if (!cross)
				world.setExcludePair(bodies[i], bodies[j]);
		}
	}
}

} // namespace

void rebuildWorld(collision::CollisionWorld& world, IRobotDocumentHost* doc, BackendDataManager& backend,
				  const RobotCollision::Settings& settings, IRobotOsgViewHost* osg)
{
	world.clear();
	world.setSecurityMarginMm(settings.securityMarginMm);
	if (!doc)
		return;

	std::vector<collision::CollisionBodyId> bodies;
	std::unordered_set<std::string> robotBackendIds;
	for (int ri = 0; ri < doc->robotKinematicInstanceCount(); ++ri)
	{
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (!doc->robotPerLinkKinematicsForInstance(ri, pl))
			continue;
		for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
		{
			robotBackendIds.insert(it.value().toStdString());
			auto data = backend.getData(it.value().toStdString());
			upsertFromBackend(world, makeLinkBody(it.value(), it.key()), data, osg, &bodies);
		}

		const QString urdf = doc->robotUrdfAbsolutePathForInstance(ri);
		QHash<QString, QString> childToParent;
		if (!urdf.isEmpty() && UrdfRobotLoader::loadLinkChildToParentMap(urdf, childToParent, nullptr))
		{
			// 近邻穿模兜底；完整自碰需 SRDF，见下方同实例全排除
			constexpr int kMaxAcmHops = 3;
			for (auto it = childToParent.constBegin(); it != childToParent.constEnd(); ++it)
			{
				const QString childLink = it.key();
				const QString childBid = pl.linkNameToBackendId.value(childLink);
				if (childBid.isEmpty())
					continue;
				QString ancLink = it.value();
				for (int hop = 1; hop <= kMaxAcmHops && !ancLink.isEmpty(); ++hop)
				{
					const QString ancBid = pl.linkNameToBackendId.value(ancLink);
					if (!ancBid.isEmpty())
					{
						world.setExcludePair(makeLinkBody(childBid, childLink), makeLinkBody(ancBid, ancLink));
					}
					ancLink = childToParent.value(ancLink);
				}
			}
		}

		// 无 SRDF 时 Fanuc 等视觉 mesh 远端连杆也会与基座穿模；同实例只检对场景，不做臂自碰
		{
			QVector<QPair<QString, QString>> links;
			links.reserve(pl.linkNameToBackendId.size());
			for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
			{
				if (!it.value().isEmpty())
					links.push_back({it.key(), it.value()});
			}
			for (int i = 0; i < links.size(); ++i)
			{
				for (int j = i + 1; j < links.size(); ++j)
				{
					world.setExcludePair(makeLinkBody(links[i].second, links[i].first),
										 makeLinkBody(links[j].second, links[j].first));
				}
			}
		}
	}

	const auto all = backend.listData();
	for (const auto& data : all)
	{
		if (!data || !data->hasGeometry())
			continue;
		const std::string bid = data->id();
		if (robotBackendIds.count(bid) > 0)
			continue;
		if (data->className() == "RobotAssembly" || data->className() == "Group")
			continue;
		upsertFromBackend(world, makeSceneBody(QString::fromStdString(bid)), data, osg, &bodies);
	}

	applyWhiteBlackListFilter(world, settings, bodies);
}

void updatePoses(collision::CollisionWorld& world, IRobotDocumentHost* doc, BackendDataManager& backend,
				 IRobotOsgViewHost* osg)
{
	if (!doc)
		return;
	for (int ri = 0; ri < doc->robotKinematicInstanceCount(); ++ri)
	{
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (!doc->robotPerLinkKinematicsForInstance(ri, pl))
			continue;
		for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
		{
			auto data = backend.getData(it.value().toStdString());
			if (!data)
				continue;
			const ResolvedBodyWorld resolved = resolveBodyWorld(osg, data);
			world.setWorldPose(makeLinkBody(it.value(), it.key()), resolved.W, resolved.poseSource);
		}
	}
	const auto all = backend.listData();
	std::unordered_set<std::string> robotIds;
	for (int ri = 0; ri < doc->robotKinematicInstanceCount(); ++ri)
	{
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (!doc->robotPerLinkKinematicsForInstance(ri, pl))
			continue;
		for (auto it = pl.linkNameToBackendId.constBegin(); it != pl.linkNameToBackendId.constEnd(); ++it)
			robotIds.insert(it.value().toStdString());
	}
	for (const auto& data : all)
	{
		if (!data || !data->hasGeometry())
			continue;
		if (robotIds.count(data->id()) > 0)
			continue;
		if (data->className() == "RobotAssembly" || data->className() == "Group")
			continue;
		const ResolvedBodyWorld resolved = resolveBodyWorld(osg, data);
		world.setWorldPose(makeSceneBody(QString::fromStdString(data->id())), resolved.W, resolved.poseSource);
	}
}

bool validateJointTrajectory(collision::CollisionWorld& world, IRobotDocumentHost* doc, BackendDataManager& backend,
							 const int instanceIndex, const QVector<double>& seedJointsBefore,
							 const std::vector<std::vector<double>>& jointTrajectoryRad,
							 const RobotCollision::Settings& settings, std::string* failSummary,
							 IRobotOsgViewHost* osg)
{
	if (!settings.enabled || !doc)
		return true;

	rebuildWorld(world, doc, backend, settings, osg);
	if (world.bodyCount() < 2)
		return true;

	std::vector<std::vector<double>> samples = jointTrajectoryRad;
	if (samples.empty() && !seedJointsBefore.isEmpty())
	{
		samples.push_back(std::vector<double>(seedJointsBefore.begin(), seedJointsBefore.end()));
	}
	if (samples.size() == 1)
	{
		// PTP 等仅终点：再插几个中间姿态不够时至少检终点；若有起点则两端
		samples.insert(samples.begin(), std::vector<double>(seedJointsBefore.begin(), seedJointsBefore.end()));
	}
	// 仅两端时按关节步长加密，避免漏检段中碰撞
	if (samples.size() == 2 && !samples[0].empty() && samples[0].size() == samples[1].size())
	{
		constexpr double kMaxStepRad = 0.05;
		double span = 0.0;
		for (std::size_t j = 0; j < samples[0].size(); ++j)
		{
			const double d = samples[1][j] - samples[0][j];
			span += d * d;
		}
		span = std::sqrt(span);
		const int steps = std::max(1, static_cast<int>(std::ceil(span / kMaxStepRad)));
		if (steps > 1)
		{
			std::vector<std::vector<double>> dense;
			dense.reserve(static_cast<std::size_t>(steps) + 1);
			dense.push_back(samples[0]);
			for (int s = 1; s < steps; ++s)
			{
				const double t = static_cast<double>(s) / static_cast<double>(steps);
				std::vector<double> q(samples[0].size());
				for (std::size_t j = 0; j < q.size(); ++j)
					q[j] = samples[0][j] + t * (samples[1][j] - samples[0][j]);
				dense.push_back(std::move(q));
			}
			dense.push_back(samples[1]);
			samples.swap(dense);
		}
	}

	// 限流：最多 24 个样本
	constexpr std::size_t kMaxSamples = 24;
	std::vector<std::size_t> indices;
	if (samples.size() <= kMaxSamples)
	{
		indices.resize(samples.size());
		for (std::size_t i = 0; i < samples.size(); ++i)
			indices[i] = i;
	}
	else
	{
		indices.reserve(kMaxSamples);
		for (std::size_t i = 0; i < kMaxSamples; ++i)
		{
			const double t = static_cast<double>(i) / static_cast<double>(kMaxSamples - 1);
			indices.push_back(static_cast<std::size_t>(std::llround(t * static_cast<double>(samples.size() - 1))));
		}
	}

	QVector<double> restore = seedJointsBefore;
	QVector<double> agg;
	for (const std::size_t si : indices)
	{
		if (si >= samples.size())
			continue;
		const auto& q = samples[si];
		QVector<double> qv;
		qv.reserve(static_cast<int>(q.size()));
		for (double v : q)
			qv.push_back(v);
		if (!doc->applyJointAnglesRad(instanceIndex, qv, agg))
			continue;
		updatePoses(world, doc, backend, osg);
		const collision::CollisionQueryResult hit = world.checkAll(4);
		if (hit.inCollision)
		{
			(void)doc->applyJointAnglesRad(instanceIndex, restore, agg);
			if (failSummary)
			{
				*failSummary = hit.summary.empty() ? "Collision detected along trajectory" : hit.summary;
				*failSummary += " (sample " + std::to_string(si) + ")";
			}
			return false;
		}
	}
	(void)doc->applyJointAnglesRad(instanceIndex, restore, agg);
	return true;
}

} // namespace BackendCollisionSync
