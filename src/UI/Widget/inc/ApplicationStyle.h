#ifndef WIDGET_APPLICATIONSTYLE_H
#define WIDGET_APPLICATIONSTYLE_H

/// @file ApplicationStyle.h
/// @brief 应用外观：浅色与深色主题的加载、保存与应用（QApplication）

#include "widget_global.h"

class QApplication;

/// 应用外观：浅色与深色主题的加载、保存与应用（QApplication）
namespace ApplicationStyle
{
enum class Theme
{
	Light,
	Dark
};

WIDGET_EXPORT void applyTheme(QApplication* app, Theme theme);
WIDGET_EXPORT Theme loadSavedTheme();
WIDGET_EXPORT void saveTheme(Theme theme);
WIDGET_EXPORT bool usesDarkTheme();

} // namespace ApplicationStyle

#endif // WIDGET_APPLICATIONSTYLE_H
