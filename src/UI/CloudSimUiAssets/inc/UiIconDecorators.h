#ifndef CLOUDSIMUIASSETS_UIICONDECORATORS_H
#define CLOUDSIMUIASSETS_UIICONDECORATORS_H

/// @file UiIconDecorators.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief UiIconDecorators 接口

#include "uiassets_global.h"

#include "UiIconId.h"
#include "UiIcons.h"

class QAbstractButton;
class QAction;

namespace UiIconDecorators
{
enum class IconPlacement
{
	Leading,
	TextUnderIcon,
	IconOnly
};

UIASSETS_EXPORT void apply(QAbstractButton* btn, UiIconId id, IconPlacement placement = IconPlacement::Leading,
						   UiIcons::Size size = UiIcons::Size::Medium);

UIASSETS_EXPORT void apply(QAction* action, UiIconId id, UiIcons::Size size = UiIcons::Size::Medium);

UIASSETS_EXPORT void refreshAllDecorated();

} // namespace UiIconDecorators

#endif // CLOUDSIMUIASSETS_UIICONDECORATORS_H
