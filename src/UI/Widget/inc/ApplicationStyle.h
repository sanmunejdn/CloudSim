#ifndef WIDGET_APPLICATIONSTYLE_H
#define WIDGET_APPLICATIONSTYLE_H

/// @file ApplicationStyle.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 应用外观：浅色与深色主题的加载、保存与应用（QApplication）

#include "widget_global.h"

#include <QColor>
#include <QString>

class QApplication;

/// 应用外观：浅色与深色主题的加载、保存与应用（QApplication）
namespace ApplicationStyle
{
enum class Theme
{
	Light,
	Dark
};

/// 全局色源；QSS / Palette / 局部高亮都从这里取，避免浅深两套字面量漂移
struct ThemeTokens
{
	QColor window;
	QColor panel;
	QColor base;
	QColor text;
	QColor textMuted;
	QColor accent;
	QColor accentHover;
	QColor accentPressed;
	QColor selectionBg;
	QColor selectionText;
	QColor hoverBg;
	QColor danger;
	QColor border;
	QColor borderStrong;
	QColor button;
	QColor buttonHover;
	int radiusSmPx = 4;
};

WIDGET_EXPORT ThemeTokens tokens(Theme theme);
WIDGET_EXPORT void applyTheme(QApplication* app, Theme theme);
WIDGET_EXPORT Theme loadSavedTheme();
WIDGET_EXPORT void saveTheme(Theme theme);
WIDGET_EXPORT bool usesDarkTheme();

} // namespace ApplicationStyle

#endif // WIDGET_APPLICATIONSTYLE_H
