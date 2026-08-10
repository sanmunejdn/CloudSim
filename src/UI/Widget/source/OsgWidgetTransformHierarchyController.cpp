/// @file OsgWidgetTransformHierarchyController.cpp
/// @brief OsgWidgetTransformHierarchyController 实现

#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendIdUserData.h"
#include "OsgWidget.h"
#include "OsgWidgetTransformHierarchyController.h"

#include <osg/MatrixTransform>
#include <osg/Quat>
#include <osg/Vec3>
#include <osg/Vec3d>

namespace
{
void removeFromAllParents(osg::Node* node)
{
	if (!node)
	{
		return;
	}
	while (node->getNumParents() > 0)
	{
		node->getParent(0)->removeChild(node);
	}
}
} // namespace

std::vector<osg::ref_ptr<osg::MatrixTransform>>
OsgWidgetTransformHierarchyController::detachChildBackendRoots(OsgWidget& self, const std::string& parentBackendId)
{
	std::vector<osg::ref_ptr<osg::MatrixTransform>> children;
	if (parentBackendId.empty())
	{
		return children;
	}
	for (const auto& kv : self.m_backendParentIds)
	{
		if (kv.second != parentBackendId)
		{
			continue;
		}
		auto it = self.m_backendObjectRoots.find(kv.first);
		if (it == self.m_backendObjectRoots.end() || !it->second.valid())
		{
			continue;
		}
		removeFromAllParents(it->second.get());
		children.push_back(it->second);
	}
	return children;
}

void OsgWidgetTransformHierarchyController::reattachChildBackendRoots(
	osg::MatrixTransform* newParent, const std::vector<osg::ref_ptr<osg::MatrixTransform>>& children)
{
	if (!newParent)
	{
		return;
	}
	for (const osg::ref_ptr<osg::MatrixTransform>& child : children)
	{
		if (child.valid())
		{
			newParent->addChild(child.get());
		}
	}
}

void OsgWidgetTransformHierarchyController::placeBackendOuterInScene(OsgWidget& self, const std::string& backendId,
																	osg::MatrixTransform* outer)
{
	if (!outer)
	{
		return;
	}
	removeFromAllParents(outer);
	std::string logicalParent;
	const auto pit = self.m_backendParentIds.find(backendId);
	if (pit != self.m_backendParentIds.end())
	{
		logicalParent = pit->second;
	}
	if (!logicalParent.empty())
	{
		auto parentIt = self.m_backendObjectRoots.find(logicalParent);
		if (parentIt != self.m_backendObjectRoots.end() && parentIt->second.valid())
		{
			parentIt->second->addChild(outer);
			return;
		}
	}
	if (self.m_backendObjectsGroup.valid())
	{
		self.m_backendObjectsGroup->addChild(outer);
	}
}

void OsgWidgetTransformHierarchyController::setBackendParent(OsgWidget& self, const std::string& backendId,
															 const std::string& parentBackendId)
{
	if (backendId.empty())
	{
		return;
	}

	auto childIt = self.m_backendObjectRoots.find(backendId);
	const bool haveChildPat = (childIt != self.m_backendObjectRoots.end() && childIt->second.valid());

	if (parentBackendId.empty())
	{
		const auto pit = self.m_backendParentIds.find(backendId);
		if (pit == self.m_backendParentIds.end() && haveChildPat)
		{
			osg::MatrixTransform* childMt = childIt->second.get();
			if (self.m_backendObjectsGroup.valid() && childMt->getNumParents() == 1 &&
				childMt->getParent(0) == self.m_backendObjectsGroup.get())
			{
				return;
			}
		}
		osg::Matrixd savedWorld;
		const bool haveWorld = self.getBackendRootWorldMatrix(backendId, savedWorld);
		self.m_backendParentIds.erase(backendId);
		if (haveChildPat)
		{
			osg::MatrixTransform* childMt = childIt->second.get();
			removeFromAllParents(childMt);
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

	auto parentIt = self.m_backendObjectRoots.find(parentBackendId);
	const auto pit = self.m_backendParentIds.find(backendId);
	if (pit != self.m_backendParentIds.end() && pit->second == parentBackendId && haveChildPat &&
		parentIt != self.m_backendObjectRoots.end() && parentIt->second.valid())
	{
		osg::MatrixTransform* childMt = childIt->second.get();
		if (childMt->getNumParents() == 1 && childMt->getParent(0) == parentIt->second.get())
		{
			return;
		}
	}

	osg::Matrixd savedWorld;
	const bool haveWorld = self.getBackendRootWorldMatrix(backendId, savedWorld);

	self.m_backendParentIds[backendId] = parentBackendId;

	if (!haveChildPat)
	{
		return;
	}
	osg::MatrixTransform* childMt = childIt->second.get();

	if (parentIt == self.m_backendObjectRoots.end() || !parentIt->second.valid())
	{
		if (haveWorld)
		{
			self.setBackendRootWorldMatrixFromWorld(backendId, savedWorld);
		}
		return;
	}

	osg::MatrixTransform* parentMt = parentIt->second.get();
	removeFromAllParents(childMt);
	parentMt->addChild(childMt);

	// Do not restore pre-reparent world here: flat layout world != hierarchical local.
	// URDF / FK callers reapply mesh-world transforms in topo order after parenting.
	(void)haveWorld;
	(void)savedWorld;
}

void OsgWidgetTransformHierarchyController::removeBackendObjectVisual(OsgWidget& self, const std::string& backendId)
{
	// 拖动示教罗盘 WorldPat 在场景叠加层，删挂载/祖先后须主动拆除，否则罗盘残留
	if (self.m_tcpTeachActive && !self.m_tcpTeachMountBackendId.empty() &&
		(backendId == self.m_tcpTeachMountBackendId ||
		 isBackendDescendantOf(self, self.m_tcpTeachMountBackendId, backendId)))
	{
		self.endTcpDragTeach();
		emit self.tcpDragTeachEnded();
	}
	// 先把仍挂在本节点下的子 backend 挂回场景根，避免随父节点一起从场景消失
	const std::vector<osg::ref_ptr<osg::MatrixTransform>> children = detachChildBackendRoots(self, backendId);
	for (const osg::ref_ptr<osg::MatrixTransform>& child : children)
	{
		if (child.valid() && self.m_backendObjectsGroup.valid())
		{
			self.m_backendObjectsGroup->addChild(child.get());
		}
	}
	auto it = self.m_backendObjectRoots.find(backendId);
	if (it != self.m_backendObjectRoots.end() && it->second.valid())
	{
		removeFromAllParents(it->second.get());
	}
	self.m_backendObjectRoots.erase(backendId);
	self.unbindBackendVisualRoot(backendId);
	self.m_backendParentIds.erase(backendId);
	for (auto itp = self.m_backendParentIds.begin(); itp != self.m_backendParentIds.end();)
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

bool OsgWidgetTransformHierarchyController::isBackendDescendantOf(const OsgWidget& self, const std::string& backendId,
																  const std::string& ancestorId)
{
	return self.OsgScene::isBackendDescendantOf(backendId, ancestorId);
}

void OsgWidgetTransformHierarchyController::cacheSelectionGizmoPose(OsgWidget& self)
{
	self.cacheSelectionGizmoPose();
}

void OsgWidgetTransformHierarchyController::finalizeSelectionSync(OsgWidget& self)
{
	self.refreshAnnotationTexts();
	self.setSelectionActive(true);
	if (self.m_viewer.valid())
	{
		self.m_viewer->setSceneData(self.m_root.get());
	}
	self.requestRedraw();
}

void OsgWidgetTransformHierarchyController::syncSelectionForBackendId(OsgWidget& self, const std::string& backendId)
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
		self.attachGizmoOverlayToActiveBackend();
		self.cacheSelectionGizmoPose();
		self.syncCompassGizmoOrientation();
		osg::Node* pickNode = backendVisualResolvePickNode(self.m_activeBackendOuterPat.get());
		if (pickNode)
		{
			self.cachePickablePointsFromNode(pickNode);
		}
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

	cacheSelectionGizmoPose(self);
	finalizeSelectionSync(self);
}

void OsgWidgetTransformHierarchyController::syncActiveBackendRootFromSelectedTransform(OsgWidget& self)
{
	self.syncActiveBackendRootFromSelectedTransform();
}
