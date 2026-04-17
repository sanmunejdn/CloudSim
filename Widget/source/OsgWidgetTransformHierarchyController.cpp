#include "OsgWidgetTransformHierarchyController.h"

#include "OsgWidget.h"

#include <osg/PositionAttitudeTransform>
#include <osg/Quat>
#include <osg/Vec3>

void OsgWidgetTransformHierarchyController::setBackendParent(
	OsgWidget& self,
	const std::string& backendId,
	const std::string& parentBackendId)
{
	if (backendId.empty())
	{
		return;
	}
	if (parentBackendId.empty())
	{
		self.m_backendParentIds.erase(backendId);
		return;
	}
	self.m_backendParentIds[backendId] = parentBackendId;
}

void OsgWidgetTransformHierarchyController::removeBackendObjectVisual(
	OsgWidget& self,
	const std::string& backendId)
{
	auto it = self.m_backendObjectRoots.find(backendId);
	if (it != self.m_backendObjectRoots.end() && it->second.valid())
	{
		osg::Node* const pat = it->second.get();
		if (pat->getNumParents() > 0)
		{
			pat->getParent(0)->removeChild(pat);
		}
	}
	self.m_backendObjectRoots.erase(backendId);
	self.m_backendParentIds.erase(backendId);
	for (auto itp = self.m_backendParentIds.begin(); itp != self.m_backendParentIds.end(); )
	{
		if (itp->second == backendId)
		{
			itp = self.m_backendParentIds.erase(itp);
		}
		else
		{
			++itp;
		}
	}
	self.m_backendModelCenters.erase(backendId);
	self.m_backendVisibility.erase(backendId);
	self.m_litMeshBackendIds.erase(backendId);
	if (self.m_activeBackendId == backendId)
	{
		self.m_activeBackendId.clear();
		self.m_activeBackendOuterPat = nullptr;
	}
}

bool OsgWidgetTransformHierarchyController::isBackendDescendantOf(
	const OsgWidget& self,
	const std::string& backendId,
	const std::string& ancestorId)
{
	return self.OsgScene::isBackendDescendantOf(backendId, ancestorId);
}

void OsgWidgetTransformHierarchyController::cacheSelectionPoseFromSelectedTransform(OsgWidget& self)
{
	self.OsgScene::cacheSelectionPoseFromSelectedTransform();
}

void OsgWidgetTransformHierarchyController::finalizeSelectionSync(OsgWidget& self)
{
	self.refreshAnnotationTexts();
	self.setSelectionActive(true);
	if (self.m_viewer.valid())
	{
		self.m_viewer->setSceneData(self.m_root.get());
	}
	if (self.m_glWidget)
	{
		self.m_glWidget->update();
	}
}

void OsgWidgetTransformHierarchyController::syncSelectionForBackendId(
	OsgWidget& self,
	const std::string& backendId)
{
	self.m_activeBackendId = backendId;
	auto it = self.m_backendObjectRoots.find(backendId);
	if (it != self.m_backendObjectRoots.end() && it->second.valid())
	{
		self.m_activeBackendOuterPat = it->second;
		const auto cIt = self.m_backendModelCenters.find(backendId);
		if (cIt != self.m_backendModelCenters.end())
		{
			self.m_modelCenter = cIt->second;
		}
		if (self.m_selectedTransform.valid())
		{
			self.m_selectedTransform->setPosition(it->second->getPosition());
			self.m_selectedTransform->setAttitude(it->second->getAttitude());
		}
		self.updateCompassLocalOffsetForModelOrigin();
		self.syncCompassGizmoOrientation();
	}
	else
	{
		// Assembly/group parent without direct geometry: clear stale child cache,
		// then point picking falls back to ray-hit in whole visible scene.
		self.m_activeBackendOuterPat = nullptr;
		self.m_pickablePointsLocal.clear();
		self.m_pickablePointsPreviewLocal.clear();
		self.m_pickablePointsCenteredLocal.clear();
		self.m_kdNodes.clear();
		self.m_kdRoot = -1;
	}

	cacheSelectionPoseFromSelectedTransform(self);
	finalizeSelectionSync(self);
}

void OsgWidgetTransformHierarchyController::syncActiveBackendRootFromSelectedTransform(OsgWidget& self)
{
	self.OsgScene::syncActiveBackendRootFromSelectedTransform();
}
