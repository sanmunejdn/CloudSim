#include "pch.h"
#include "OsgScene.h"

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

PickResult OsgScene::queryPick(const PickQuery& query)
{
	PickResult out;
	const double hitRadius = query.hitRadiusPx > 0.0 ? query.hitRadiusPx : kPointPickHitRadiusPx;

	switch (query.kind)
	{
	case PickKind::PointCloud:
	{
		osg::Vec3f worldPoint;
		double distPx = 0.0;
		if (!pickNearestPointAtScreenPos(query.screenX, query.screenY, worldPoint, distPx, query.hoverPick))
		{
			return out;
		}
		out.screenDistancePx = distPx;
		out.worldPoint = worldPoint;
		out.hit = distPx <= hitRadius;
		out.backendId = m_activeBackendId;
		if (const BackendPickBundle* bundle = m_backendPickIndexes.find(m_activeBackendId))
		{
			out.indexGeneration = bundle->generation;
		}
		return out;
	}
	case PickKind::MeshFace:
	{
		osg::Vec3f p, a, b, c, n;
		std::vector<osg::Vec3f> merged;
		const std::string* scope = query.scopeBackendId.empty() ? nullptr : &query.scopeBackendId;
		if (!pickMeshFaceByRayIntersection(
				query.screenX, query.screenY, p, a, b, c, n, &merged, scope))
		{
			return out;
		}
		out.hit = true;
		out.worldPoint = p;
		out.meshNormalWorld = n;
		out.meshFaceVertsWorld = std::move(merged);
		out.backendId = query.scopeBackendId.empty() ? m_activeBackendId : query.scopeBackendId;
		return out;
	}
	case PickKind::MeshEdge:
	{
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
