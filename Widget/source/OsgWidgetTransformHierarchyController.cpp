#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "OsgWidgetTransformHierarchyController.h"

#include "OsgWidget.h"

#include <osg/MatrixTransform>
#include <osg/Quat>
#include <osg/Vec3>
#include <osg/Vec3d>

void OsgWidgetTransformHierarchyController::setBackendParent(
	OsgWidget& self,
	const std::string& backendId,
	const std::string& parentBackendId)
{
	if (backendId.empty())
	{
		return;
	}

	osg::Matrixd savedWorld;
	const bool haveWorld = self.getBackendRootWorldMatrix(backendId, savedWorld);

	auto childIt = self.m_backendObjectRoots.find(backendId);
	const bool haveChildPat = (childIt != self.m_backendObjectRoots.end() && childIt->second.valid());

	if (parentBackendId.empty())
	{
		self.m_backendParentIds.erase(backendId);
		if (haveChildPat)
		{
			osg::MatrixTransform* childMt = childIt->second.get();
			while (childMt->getNumParents() > 0)
			{
				childMt->getParent(0)->removeChild(childMt);
			}
			if (self.m_backendObjectsGroup.valid())
			{
				self.m_backendObjectsGroup->addChild(childMt);
			}
		}
		if (haveWorld && haveChildPat)
		{
			self.setBackendRootWorldMatrixFromWorld(backendId, savedWorld);
		}
		return;
	}

	if (parentBackendId == backendId)
	{
		return;
	}
	if (self.isBackendDescendantOf(parentBackendId, backendId))
	{
		return;
	}

	self.m_backendParentIds[backendId] = parentBackendId;

	if (!haveChildPat)
	{
		return;
	}
	osg::MatrixTransform* childMt = childIt->second.get();

	auto parentIt = self.m_backendObjectRoots.find(parentBackendId);
	if (parentIt == self.m_backendObjectRoots.end() || !parentIt->second.valid())
	{
		if (haveWorld)
		{
			self.setBackendRootWorldMatrixFromWorld(backendId, savedWorld);
		}
		return;
	}

	osg::MatrixTransform* parentMt = parentIt->second.get();
	while (childMt->getNumParents() > 0)
	{
		childMt->getParent(0)->removeChild(childMt);
	}
	parentMt->addChild(childMt);

	if (haveWorld)
	{
		self.setBackendRootWorldMatrixFromWorld(backendId, savedWorld);
	}
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
	self.unbindBackendVisualRoot(backendId);
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
			osg::Vec3d t;
			osg::Quat r;
			osg::Vec3d s;
			osg::Quat so;
			it->second->getMatrix().decompose(t, r, s, so);
			self.m_selectedTransform->setPosition(osg::Vec3f(static_cast<float>(t.x()), static_cast<float>(t.y()),
				static_cast<float>(t.z())));
			self.m_selectedTransform->setAttitude(r);
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
