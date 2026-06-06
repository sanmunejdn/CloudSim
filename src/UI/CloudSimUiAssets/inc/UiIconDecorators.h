#pragma once

#include "UiIconId.h"
#include "UiIcons.h"
#include "uiassets_global.h"

class QAbstractButton;
class QAction;

namespace UiIconDecorators {

enum class IconPlacement
{
	Leading,
	TextUnderIcon,
	IconOnly
};

UIASSETS_EXPORT void apply(
	QAbstractButton* btn,
	UiIconId id,
	IconPlacement placement = IconPlacement::Leading,
	UiIcons::Size size = UiIcons::Size::Medium);

UIASSETS_EXPORT void apply(QAction* action, UiIconId id, UiIcons::Size size = UiIcons::Size::Medium);

UIASSETS_EXPORT void refreshAllDecorated();

} // namespace UiIconDecorators
