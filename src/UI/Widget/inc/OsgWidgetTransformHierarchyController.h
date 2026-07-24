#ifndef WIDGET_OSGWIDGETTRANSFORMHIERARCHYCONTROLLER_H
#define WIDGET_OSGWIDGETTRANSFORMHIERARCHYCONTROLLER_H

/// @file OsgWidgetTransformHierarchyController.h
/// @brief 后端父子层级与联动变换控制（自 OsgWidget 拆出

#include <osg/MatrixTransform>
#include <osg/ref_ptr>
#include <string>
#include <vector>

class OsgWidget;

/// 后端父子层级与联动变换控制（自 OsgWidget 拆出
class OsgWidgetTransformHierarchyController
{
public:
	static void setBackendParent(OsgWidget& self, const std::string& backendId, const std::string& parentBackendId);
	static void removeBackendObjectVisual(OsgWidget& self, const std::string& backendId);
	static bool isBackendDescendantOf(const OsgWidget& self, const std::string& backendId,
									  const std::string& ancestorId);
	static void cacheSelectionGizmoPose(OsgWidget& self);
	static void finalizeSelectionSync(OsgWidget& self);
	static void syncSelectionForBackendId(OsgWidget& self, const std::string& backendId);
	static void syncActiveBackendRootFromSelectedTransform(OsgWidget& self);

	/// upsert 前拆下逻辑子节点，避免换 outer 时整棵子树被带走成孤儿
	static std::vector<osg::ref_ptr<osg::MatrixTransform>> detachChildBackendRoots(OsgWidget& self,
																				   const std::string& parentBackendId);
	static void reattachChildBackendRoots(osg::MatrixTransform* newParent,
										  const std::vector<osg::ref_ptr<osg::MatrixTransform>>& children);
	/// 从任意父节点摘下后挂到逻辑父或 backendObjectsGroup
	static void placeBackendOuterInScene(OsgWidget& self, const std::string& backendId, osg::MatrixTransform* outer);
};

#endif // WIDGET_OSGWIDGETTRANSFORMHIERARCHYCONTROLLER_H
