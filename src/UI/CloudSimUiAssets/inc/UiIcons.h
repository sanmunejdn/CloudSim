#pragma once

#include "UiIconId.h"
#include "uiassets_global.h"

class QIcon;

namespace UiIcons {

enum class Theme
{
	Light,
	Dark,
	Auto
};

enum class Size
{
	Small = 16,
	Medium = 24
};

UIASSETS_EXPORT QIcon icon(UiIconId id, Size size = Size::Medium, Theme theme = Theme::Auto);

UIASSETS_EXPORT void setTheme(Theme theme);
UIASSETS_EXPORT Theme currentTheme();
UIASSETS_EXPORT void invalidateCache();

} // namespace UiIcons
