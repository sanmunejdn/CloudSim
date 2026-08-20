#ifndef CLOUDSIMHOST_SELECTIONVISUALSERVICE_H
#define CLOUDSIMHOST_SELECTIONVISUALSERVICE_H

/// @file SelectionVisualService.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 选中对象可视化加载（从 MainWindowSelectionService 下沉）

#include "cloudsim_host_global.h"

#include "CoreTypes.h"

namespace cloudsim::host
{
class DocumentHost;

/// 选中对象可视化加载（从 MainWindowSelectionService 下沉）
class CLOUDSIM_HOST_EXPORT SelectionVisualService
{
public:
	static void ensureSelectionVisual(DocumentHost& host, const core::ObjectId& backendId, bool urdfLinkMesh = false);
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_SELECTIONVISUALSERVICE_H
