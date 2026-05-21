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

#include "ObjectGizmoFrame.h"

#include <algorithm>
#include <cmath>

#include "BackendDataBase.h"
#include "BackendGeometryMetrics.h"
#include "BackendIdUserData.h"
#include "BackendVisualRegistry.h"

#include <osg/Group>
#include <osg/Node>
#include <osg/Matrixd>
#include <osg/MatrixTransform>
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
	if (!m_gizmoOverlayGroup.valid() || !m_activeBackendOuterPat.valid() || m_activeBackendOuterPat->getNumChildren() < 1)
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

	ObjectGizmoFrame frame;
	if (it != m_backendObjectRoots.end() && it->second.valid())
	{
		osg::MatrixTransform* const outer = it->second.get();
		const auto parentRel = m_backendParentIds.find(id);
		const bool hasBackendParent =
			parentRel != m_backendParentIds.end() && !parentRel->second.empty();
		// Hierarchical children (URDF links, STEP/DXF children): backend pose() is decomposed
		// world values; applyToOuter would treat them as root-local and jump the mesh on select.
		if (hasBackendParent && ObjectGizmoFrame::fromOuter(outer, m_modelCenter, frame))
		{
			attachGizmoOverlayToActiveBackend();
			cacheSelectionGizmoPose();
		}
		else
		{
			const BackendVec3 p = data.pose();
			const BackendVec3 r = data.rotation();
			const osg::Vec3f pose(static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z));
			const osg::Quat q = eulerDegToQuat(osg::Vec3f(static_cast<float>(r.x), static_cast<float>(r.y), static_cast<float>(r.z)));
			frame.setFromBackend(pose, q, m_modelCenter);
			frame.applyToOuter(outer);
			attachGizmoOverlayToActiveBackend();
			syncActiveBackendRootFromObjectFrame(frame, false);
			cacheSelectionGizmoPose();
		}
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
