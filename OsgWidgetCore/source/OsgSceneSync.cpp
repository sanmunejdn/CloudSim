#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "OsgScene.h"

#include <algorithm>
#include <cmath>

#include "BackendDataBase.h"
#include "BackendGeometryMetrics.h"
#include "BackendIdUserData.h"
#include "BackendVisualRegistry.h"

#include <osg/Node>
#include <osg/PositionAttitudeTransform>

osg::Vec3f OsgScene::computePointCloudCenterFromXyz(const std::vector<float>& xyz) const
{
	return backend_geometry_metrics::pointCloudCenterFromXyz(xyz);
}

float OsgScene::computePointCloudDiagonalFromXyz(const std::vector<float>& xyz) const
{
	return backend_geometry_metrics::pointCloudDiagonalFromXyz(xyz);
}

osg::Vec3f OsgScene::computeMeshCenterFromSoup(const std::vector<float>& soup) const
{
	return backend_geometry_metrics::meshCenterFromSoup(soup);
}

float OsgScene::computeMeshDiagonalFromSoup(const std::vector<float>& soup) const
{
	return backend_geometry_metrics::meshDiagonalFromSoup(soup);
}

void OsgScene::syncGizmoAndPickFromBackend(const BackendDataBase& data)
{
	const std::string id = data.id();
	m_activeBackendId = id;
	auto it = m_backendObjectRoots.find(id);
	if (it != m_backendObjectRoots.end() && it->second.valid())
	{
		m_activeBackendOuterPat = it->second;
	}
	else
	{
		m_activeBackendOuterPat = nullptr;
	}
	osg::Vec3f computedCenter{};
	float computedDiagonal = 1.0f;
	BackendVisualRegistry::computeModelCenterAndDiagonal(data, computedCenter, computedDiagonal);
	auto cIt = m_backendModelCenters.find(id);
	if (cIt != m_backendModelCenters.end())
	{
		m_modelCenter = cIt->second;
	}
	else
	{
		m_modelCenter = computedCenter;
		m_backendModelCenters[id] = m_modelCenter;
	}
	m_activeModelDiagonal = computedDiagonal;
	if (m_selectedTransform.valid())
	{
		if (it != m_backendObjectRoots.end() && it->second.valid())
		{
			osg::PositionAttitudeTransform* outer = it->second.get();
			m_selectedTransform->setPosition(outer->getPosition());
			m_selectedTransform->setAttitude(outer->getAttitude());
		}
		else
		{
			const BackendVec3 p = data.pose();
			const BackendVec3 r = data.rotation();
			const osg::Vec3f pose(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
			m_selectedTransform->setPosition(m_modelCenter + pose);
			m_selectedTransform->setAttitude(eulerDegToQuat(osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z))));
		}
	}
	syncActiveBackendRootFromSelectedTransform();
	osg::Node* pickNode = nullptr;
	if (m_activeBackendOuterPat.valid())
	{
		pickNode = backendVisualResolvePickNode(m_activeBackendOuterPat.get());
	}
	if (pickNode)
	{
		cachePickablePointsFromNode(pickNode);
	}
	updateCompassLocalOffsetForModelOrigin();
}
