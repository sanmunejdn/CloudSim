#pragma once

#include <string>

class OsgWidget;

/// 后端父子层级与联动变换控制（从 OsgWidget 拆出）。
class OsgWidgetTransformHierarchyController
{
public:
	static void setBackendParent(OsgWidget& self, const std::string& backendId, const std::string& parentBackendId);
	static void removeBackendObjectVisual(OsgWidget& self, const std::string& backendId);
	static bool isBackendDescendantOf(const OsgWidget& self, const std::string& backendId, const std::string& ancestorId);
	static void cacheSelectionPoseFromSelectedTransform(OsgWidget& self);
	static void finalizeSelectionSync(OsgWidget& self);
	static void syncSelectionForBackendId(OsgWidget& self, const std::string& backendId);
	static void syncActiveBackendRootFromSelectedTransform(OsgWidget& self);
};
