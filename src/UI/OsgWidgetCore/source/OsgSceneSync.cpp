/// @file OsgSceneSync.cpp
/// @brief OsgSceneSync 实现

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendDataBase.h"
#include "BackendGeometryMetrics.h"
#include "BackendIdUserData.h"
#include "BackendVisualRegistry.h"
#include "ObjectGizmoFrame.h"
#include "OsgScene.h"

#include <algorithm>
#include <cmath>

#include <osg/Group>
#include <osg/MatrixTransform>
#include <osg/Matrixd>
#include <osg/Node>
#include <osg/Quat>
#include <osg/Vec3d>

bool OsgScene::readActiveObjectGizmoFrame(ObjectGizmoFrame& out) const
{
	if (!m_activeBackendOuterPat.valid())
	{
		return false;
	}
	if (!ObjectGizmoFrame::fromOuter(m_activeBackendOuterPat.get(), m_modelCenter, out))
	{
		return false;
	}
	m_modelCenter = out.modelCenter();
	return true;
}

void OsgScene::detachGizmoOverlay()
{
	if (m_gizmoOverlayGroup.valid() && m_gizmoAttachedInner.valid())
	{
		osg::Group* innerG = m_gizmoAttachedInner->asGroup();
		if (innerG)
		{
			innerG->removeChild(m_gizmoOverlayGroup.get());
		}
		m_gizmoAttachedInner = nullptr;
	}
}

void OsgScene::attachGizmoOverlayToActiveBackend()
{
	detachGizmoOverlay();
	if (!m_gizmoOverlayGroup.valid() || !m_activeBackendOuterPat.valid() ||
		m_activeBackendOuterPat->getNumChildren() < 1)
	{
		return;
	}
	osg::Node* inner = m_activeBackendOuterPat->getChild(0);
	osg::Group* innerG = inner ? inner->asGroup() : nullptr;
	if (!innerG)
	{
		return;
	}
	innerG->addChild(m_gizmoOverlayGroup.get());
	m_gizmoAttachedInner = inner;
}

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

	if (it != m_backendObjectRoots.end() && it->second.valid())
	{
		// outer 已是 worldMatrix；选中/加载禁止用欧拉 pose 回写，否则换选会带动其它件
		attachGizmoOverlayToActiveBackend();
		cacheSelectionGizmoPose();
	}
	else
	{
		detachGizmoOverlay();
	}

	osg::Node* pickNode = nullptr;
	if (m_activeBackendOuterPat.valid())
	{
		pickNode = backendVisualResolvePickNode(m_activeBackendOuterPat.get());
	}
	if (pickNode)
	{
		cachePickablePointsFromNode(pickNode);
	}
	if (m_selectionActive && m_objectSelectionMode)
	{
		attachCompassGraphics();
	}
	syncCompassGizmoOrientation();
	logGizmoPivotDiagnostics("syncGizmoAndPickFromBackend");
}
