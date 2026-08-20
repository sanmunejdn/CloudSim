#ifndef WIDGET_APPLICATIONSETTINGS_H
#define WIDGET_APPLICATIONSETTINGS_H

/// @file ApplicationSettings.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 应用级 UI 偏好持久化（语言、主题、面板可见性、工作区模式）

#include "ApplicationStyle.h"
#include "widget_global.h"

#include <QHash>
#include <QString>

class QWidget;

namespace ApplicationSettings
{
/// 侧栏页签稳定键（与语言无关）
inline constexpr const char* kAiAssistantTabKey = "CloudSim_AiAssistant";

struct UiPreferences
{
	QString language = QStringLiteral("zh");
	ApplicationStyle::Theme theme = ApplicationStyle::Theme::Light;
	bool leftPanelVisible = true;
	bool rightPanelVisible = true;
	QHash<QString, bool> sidePanelTabs;
	QString workspaceModeId;
	int leftDockWidth = 240;
	int rightDockWidth = 360;
};

WIDGET_EXPORT QString settingsFilePath();
WIDGET_EXPORT UiPreferences load();
WIDGET_EXPORT void save(const UiPreferences& prefs);
WIDGET_EXPORT void saveTheme(ApplicationStyle::Theme theme);
WIDGET_EXPORT ApplicationStyle::Theme loadTheme();

WIDGET_EXPORT QString sidePanelTabKey(const QWidget* widget);
WIDGET_EXPORT bool sidePanelTabVisible(const QString& key, bool defaultVisible = true);

} // namespace ApplicationSettings

#endif // WIDGET_APPLICATIONSETTINGS_H
