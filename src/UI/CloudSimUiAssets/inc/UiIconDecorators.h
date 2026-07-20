#ifndef CLOUDSIMUIASSETS_UIICONDECORATORS_H
#define CLOUDSIMUIASSETS_UIICONDECORATORS_H

/// @file UiIconDecorators.h
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
