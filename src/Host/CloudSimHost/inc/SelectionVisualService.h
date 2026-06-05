#pragma once

#include "CoreTypes.h"
#include "cloudsim_host_global.h"

namespace cloudsim::host {

class DocumentHost;

/// 选中对象可视化加载（从 MainWindowSelectionService 下沉）
class CLOUDSIM_HOST_EXPORT SelectionVisualService
{
public:
	static void ensureSelectionVisual(DocumentHost& host, const core::ObjectId& backendId, bool urdfLinkMesh = false);
};

} // namespace cloudsim::host
