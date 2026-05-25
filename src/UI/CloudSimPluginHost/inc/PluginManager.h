#pragma once

#include <memory>
#include <vector>

#include <QObject>
#include <QString>
#include <QPluginLoader>

#include "ICloudSimPlugin.h"

class MainWindow;
class PluginHostContext;

/// 启动时扫描应用目录 plugins/ 并加载启用插件
class PluginManager : public QObject
{
	Q_OBJECT

public:
	explicit PluginManager(MainWindow* mainWindow, QObject* parent = nullptr);
	~PluginManager() override;

	void loadAllFromPluginsDirectory();
	void shutdownAll();
	void notifyLanguageChanged();

	QString loadSummary() const { return m_loadSummary; }

private:
	struct LoadedPlugin
	{
		QString id;
		QString displayName;
		std::unique_ptr<QPluginLoader> loader;
		ICloudSimPlugin* instance = nullptr;
	};

	bool loadOnePlugin(const QString& pluginDir, const QString& manifestPath);
	static bool parseHostVersionString(const QString& versionStr, unsigned int& outPacked);
	static bool hostVersionSatisfies(const QString& minHostVersionStr);

	MainWindow* m_mainWindow = nullptr;
	std::unique_ptr<PluginHostContext> m_hostContext;
	std::vector<std::unique_ptr<LoadedPlugin>> m_plugins;
	QString m_loadSummary;
};
