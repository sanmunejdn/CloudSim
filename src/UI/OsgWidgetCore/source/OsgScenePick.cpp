#include "pch.h"
#include "OsgScene.h"

#include "BackendIdUserData.h"
#include "BackendPickDomain.h"
#include "PickSpatialIndex.h"

const PickSpatialIndex* OsgScene::activePointPickIndex() const
{
	if (m_activeBackendId.empty())
	{
		return nullptr;
	}
	if (const BackendPickBundle* bundle = m_backendPickIndexes.find(m_activeBackendId))
	{
		if (!bundle->pointIndex.empty())
		{
			return &bundle->pointIndex;
		}
	}
	return nullptr;
}

void OsgScene::importPickSpatialIndexForActiveBackend(const PickSpatialIndex& index)
{
	m_pickablePointsLocal = index.pointsLocal();
	m_pickablePointsPreviewLocal.clear();
	m_pickablePointsCenteredLocal.clear();
	m_kdNodes.clear();
	m_kdRoot = -1;
	if (m_pickablePointsLocal.empty())
	{
		return;
	}
	try
	{
		m_pickablePointsCenteredLocal.reserve(m_pickablePointsLocal.size());
		for (const osg::Vec3f& p : m_pickablePointsLocal)
		{
			m_pickablePointsCenteredLocal.push_back(p - m_modelCenter);
		}
		rebuildPointKdTree();
	}
	catch (...)
	{
		m_pickablePointsCenteredLocal.clear();
		m_kdNodes.clear();
		m_kdRoot = -1;
	}
}

void OsgScene::nearestCandidatesByPickIndex(
	const PickSpatialIndex& index,
	const osg::Vec3f& queryLocalCentered,
	int k,
	std::vector<int>& outIndices) const
{
	index.nearestCandidates(queryLocalCentered, k, outIndices);
}

void OsgScene::setPickVisualAlias(const std::string& logicalBackendId, const std::string& visualBackendId)
{
	if (logicalBackendId.empty() || visualBackendId.empty())
	{
		return;
	}
	m_pickVisualAliases[logicalBackendId] = visualBackendId;
}

std::string OsgScene::resolvePickScopeBackendId(const std::string& backendId) const
{
	if (backendId.empty())
	{
		return {};
	}
	const auto aliasIt = m_pickVisualAliases.find(backendId);
	if (aliasIt != m_pickVisualAliases.end())
	{
		return aliasIt->second;
	}
	if (m_backendObjectRoots.find(backendId) != m_backendObjectRoots.end())
	{
		return backendId;
	}
	std::string soleBrepVisual;
	for (const auto& entry : m_backendObjectRoots)
	{
		if (!entry.second.valid())
		{
			continue;
		}
		const auto* meta = dynamic_cast<const BackendIdUserData*>(entry.second->getUserData());
		if (!meta || meta->pickDomain() != BackendPickDomain::Brep)
		{
			continue;
		}
		if (!soleBrepVisual.empty())
		{
			soleBrepVisual.clear();
			break;
		}
		soleBrepVisual = entry.first;
	}
	return soleBrepVisual.empty() ? backendId : soleBrepVisual;
}

PickResult OsgScene::queryPick(const PickQuery& queryIn)
{
	PickQuery query = queryIn;
	if (!query.scopeBackendId.empty())
	{
		query.scopeBackendId = resolvePickScopeBackendId(query.scopeBackendId);
	}
	else if (!m_activeBackendId.empty())
	{
		query.scopeBackendId = resolvePickScopeBackendId(m_activeBackendId);
	}

	PickResult out;
	const double hitRadius = query.hitRadiusPx > 0.0 ? query.hitRadiusPx : kPointPickHitRadiusPx;

	switch (query.kind)
	{
	case PickKind::PointCloud:
	{
		osg::Vec3f worldPoint;
		double distPx = 0.0;
		int pickedIdx = -1;
		if (!pickNearestPointAtScreenPos(query.screenX, query.screenY, worldPoint, distPx, query.hoverPick, &pickedIdx))
		{
			return out;
		}
		out.screenDistancePx = distPx;
		out.worldPoint = worldPoint;
		out.hit = distPx <= hitRadius;
		out.backendId = m_activeBackendId;
		out.pointIndex = pickedIdx;
		if (const BackendPickBundle* bundle = m_backendPickIndexes.find(m_activeBackendId))
		{
			out.indexGeneration = bundle->generation;
		}
		return out;
	}
	case PickKind::MeshFace:
	{
		const std::string brepScope = query.scopeBackendId.empty() ? m_activeBackendId : query.scopeBackendId;
		const bool brepBackend = isBrepPickBackend(brepScope);
		if (tryQueryBrepPick(query, true, out))
		{
			return out;
		}
		if (brepBackend)
		{
			return out;
		}
		osg::Vec3f p, a, b, c, n;
		std::vector<osg::Vec3f> merged;
		const std::string* scope = query.scopeBackendId.empty() ? nullptr : &query.scopeBackendId;
		int pickedTri = -1;
		if (!pickMeshFaceByRayIntersection(
				query.screenX, query.screenY, p, a, b, c, n, &merged, scope, &pickedTri))
		{
			return out;
		}
		out.hit = true;
		out.worldPoint = p;
		out.meshNormalWorld = n;
		out.meshFaceVertsWorld = std::move(merged);
		out.pickedTriangleIndex = pickedTri;
		out.meshTriangleIndex = pickedTri;
		out.backendId = query.scopeBackendId.empty() ? m_activeBackendId : query.scopeBackendId;
		return out;
	}
	case PickKind::MeshEdge:
	{
		const std::string brepScope = query.scopeBackendId.empty() ? m_activeBackendId : query.scopeBackendId;
		const bool brepBackend = isBrepPickBackend(brepScope);
		if (tryQueryBrepPick(query, false, out))
		{
			return out;
		}
		if (brepBackend)
		{
			return out;
		}
		osg::Vec3f p, edgeA, edgeB;
		double edgeDistPx = 0.0;
		const std::string* scope = query.scopeBackendId.empty() ? nullptr : &query.scopeBackendId;
		if (!pickMeshEdgeByRayIntersection(
				query.screenX, query.screenY, p, edgeA, edgeB, &edgeDistPx, scope))
		{
			return out;
		}
		out.hit = true;
		out.worldPoint = p;
		out.meshEdgeA = edgeA;
		out.meshEdgeB = edgeB;
		out.screenDistancePx = edgeDistPx;
		out.backendId = query.scopeBackendId.empty() ? m_activeBackendId : query.scopeBackendId;
		return out;
	}
	case PickKind::BackendObject:
	{
		if (pickAndActivateBackendAtScreenPos(query.screenX, query.screenY))
		{
			out.hit = true;
			out.backendId = m_activeBackendId;
		}
		return out;
	}
	default:
		break;
	}
	return out;
}
