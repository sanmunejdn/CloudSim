#ifndef CLOUDSIMUIASSETS_UIICONS_H
#define CLOUDSIMUIASSETS_UIICONS_H

/// @file UiIcons.h
/// @brief UiIcons 接口

#include "uiassets_global.h"

#include "UiIconId.h"

class QIcon;

namespace UiIcons
{
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

#endif // CLOUDSIMUIASSETS_UIICONS_H
