#ifndef CLOUDSIMPLUGINHOST_PLUGINMANAGER_H
#define CLOUDSIMPLUGINHOST_PLUGINMANAGER_H

/// @file PluginManager.h
/// @brief 启动时扫描应用目录 plugins/ 并加载启用插件（编译在 CloudSimHost.dll）

#include "cloudsim_host_global.h"

#include "ICloudSimPlugin.h"

#include <QObject>
#include <QPluginLoader>
#include <QString>
#include <memory>
#include <vector>

class IPluginMainWindowHost;
class PluginHostContext;
class QJsonObject;

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

	/// Widget 侧工程 IO 入口（PluginHostContext 未导出，经此转发）
	void invokeProjectAboutToSave(const QString& documentId, QJsonObject& root);
	void invokeProjectLoaded(const QString& documentId, const QJsonObject& root);
	void invokeDocumentClosed(const QString& documentId);

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

#endif // CLOUDSIMPLUGINHOST_PLUGINMANAGER_H
