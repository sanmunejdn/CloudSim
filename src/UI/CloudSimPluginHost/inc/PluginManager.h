#pragma once

#include <memory>
#include <vector>

#include <QObject>
#include <QString>
#include <QPluginLoader>

#include "cloudsim_host_global.h"
#include "ICloudSimPlugin.h"

class IPluginMainWindowHost;
class PluginHostContext;

/// 启动时扫描应用目录 plugins/ 并加载启用插件（编译在 CloudSimHost.dll）
class CLOUDSIM_HOST_EXPORT PluginManager : public QObject
{
	Q_OBJECT

public:
	explicit PluginManager(IPluginMainWindowHost* mainWindowHost, QObject* parent = nullptr);
	~PluginManager() override;

	void loadAllFromPluginsDirectory();
	void shutdownAll();
	void notifyLanguageChanged();

	QString loadSummary() const { return m_loadSummary; }

	PluginHostContext* hostContext() { return m_hostContext.get(); }
	const PluginHostContext* hostContext() const { return m_hostContext.get(); }

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
	static bool aiSdkVersionSatisfies(const QString& minAiSdkVersionStr);

	IPluginMainWindowHost* m_mainWindowHost = nullptr;
	std::unique_ptr<PluginHostContext> m_hostContext;
	std::vector<std::unique_ptr<LoadedPlugin>> m_plugins;
	QString m_loadSummary;
};
