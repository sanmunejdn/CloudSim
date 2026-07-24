/// @file BackendCollisionSync.cpp
/// @brief 后端几何 → CollisionWorld；轨迹抽样校验

#include "BackendCollisionSync.h"

#include "BackendDataManager.h"
#include "BrepBackendData.h"
#include "IRobotDocumentHost.h"
#include "MeshBackendData.h"
#include "MeshDiscretize.h"
#include "UrdfRobotLoader.h"

#include <algorithm>
#include <cmath>
#include <unordered_set>

namespace BackendCollisionSync
{
namespace
{
collision::Mat4 toCollisionMat4(const BackendMat4& m)
{
	collision::Mat4 out{};
	for (int i = 0; i < 16; ++i)
		out[static_cast<std::size_t>(i)] = m.v[i];
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
					   const std::shared_ptr<BackendDataBase>& data)
{
	if (!data || !data->hasGeometry())
		return;
	const collision::Mat4 W = toCollisionMat4(data->worldMatrix());
	if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
	{
		const auto& soup = mesh->triangleSoup();
		if (soup.size() >= 9)
			world.upsertMeshBody(id, soup.data(), soup.size(), W);
		return;
	}
	if (auto brep = std::dynamic_pointer_cast<BrepBackendData>(data))
	{
		std::vector<float> soup;
		if (discretizeBrepSoup(*brep, soup))
			world.upsertMeshBody(id, soup.data(), soup.size(), W);
	}
}

} // namespace

void rebuildWorld(collision::CollisionWorld& world, IRobotDocumentHost* doc, BackendDataManager& backend,
				  const RobotCollision::Settings& settings)
{
	world.clear();
	world.setSecurityMarginMm(settings.securityMarginMm);
	if (!doc)
		return;

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
			upsertFromBackend(world, makeLinkBody(it.value(), it.key()), data);
		}

		const QString urdf = doc->robotUrdfAbsolutePathForInstance(ri);
		QHash<QString, QString> childToParent;
		if (!urdf.isEmpty() && UrdfRobotLoader::loadLinkChildToParentMap(urdf, childToParent, nullptr))
		{
			for (auto it = childToParent.constBegin(); it != childToParent.constEnd(); ++it)
			{
				const QString childBid = pl.linkNameToBackendId.value(it.key());
				const QString parentBid = pl.linkNameToBackendId.value(it.value());
				if (childBid.isEmpty() || parentBid.isEmpty())
					continue;
				world.setExcludePair(makeLinkBody(childBid, it.key()), makeLinkBody(parentBid, it.value()));
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
		upsertFromBackend(world, makeSceneBody(QString::fromStdString(bid)), data);
	}
}

void updatePoses(collision::CollisionWorld& world, IRobotDocumentHost* doc, BackendDataManager& backend)
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
			world.setWorldPose(makeLinkBody(it.value(), it.key()), toCollisionMat4(data->worldMatrix()));
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
		world.setWorldPose(makeSceneBody(QString::fromStdString(data->id())), toCollisionMat4(data->worldMatrix()));
	}
}

bool validateJointTrajectory(collision::CollisionWorld& world, IRobotDocumentHost* doc, BackendDataManager& backend,
							 const int instanceIndex, const QVector<double>& seedJointsBefore,
							 const std::vector<std::vector<double>>& jointTrajectoryRad,
							 const RobotCollision::Settings& settings, std::string* failSummary)
{
	if (!settings.enabled || !doc)
		return true;

	rebuildWorld(world, doc, backend, settings);
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
		updatePoses(world, doc, backend);
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
