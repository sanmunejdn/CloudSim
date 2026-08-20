#ifndef CLOUDSIMUIASSETS_APPICON_H
#define CLOUDSIMUIASSETS_APPICON_H

/// @file AppIcon.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 应用级图标（Logo），独立于工具栏图标系统 UiIcons。

#include "uiassets_global.h"

class QIcon;

/// 应用级图标（Logo），独立于工具栏图标系统 UiIcons。
/// 资源路径：:/cloudsim/logo/{light|dark}/cloudsim_logo_{size}.png
namespace AppIcon
{
UIASSETS_EXPORT QIcon logo();

} // namespace AppIcon

#endif // CLOUDSIMUIASSETS_APPICON_H
