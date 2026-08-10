/// @file PluginManager.cpp
/// @brief PluginManager 实现

#include "PluginManager.h"

#include "CloudSimAiVersion.h"
#include "CloudSimPluginVersion.h"
#include "IAiAssistantHost.h"
#include "ICloudSimAiPlugin.h"
#include "ICloudSimPlugin.h"
#include "IPluginMainWindowHost.h"
#include "PluginHostContext.h"
#include "RunLogger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QPluginLoader>
#include <QStatusBar>

#include <json.hpp>

PluginManager::PluginManager(IPluginMainWindowHost* mainWindowHost, QObject* parent)
	: QObject(parent), m_mainWindowHost(mainWindowHost),
	  m_hostContext(std::make_unique<PluginHostContext>(mainWindowHost, this))
{
}

PluginManager::~PluginManager()
{
	shutdownAll();
}

void PluginManager::shutdownAll()
{
	if (m_plugins.empty())
	{
		return;
	}
	// 必须先清回调再逐个 shutdown/unload：否则 A unload 后 B::shutdown→claimWorkspaceMode 仍会调到 A 的悬空 lambda
	if (m_hostContext)
	{
		m_hostContext->prepareForPluginShutdown();
	}
	for (auto& entry : m_plugins)
	{
		if (entry && entry->instance)
		{
			if (entry->loader)
			{
				if (auto* aiPlg = qobject_cast<ICloudSimAiPlugin*>(entry->loader->instance()))
					aiPlg->shutdownAi();
			}
			entry->instance->shutdown();
		}
		if (entry && entry->loader && entry->loader->isLoaded())
		{
			entry->loader->unload();
		}
	}
	m_plugins.clear();
}

bool PluginManager::parseHostVersionString(const QString& versionStr, unsigned int& outPacked)
{
	const QStringList parts = versionStr.split(QLatin1Char('.'));
	if (parts.size() < 2)
	{
		return false;
	}
	bool okMajor = false;
	bool okMinor = false;
	const int major = parts[0].toInt(&okMajor);
	const int minor = parts[1].toInt(&okMinor);
	if (!okMajor || !okMinor || major < 0 || minor < 0)
	{
		return false;
	}
	outPacked = (static_cast<unsigned int>(major) << 16) | static_cast<unsigned int>(minor);
	return true;
}

bool PluginManager::hostVersionSatisfies(const QString& minHostVersionStr)
{
	if (minHostVersionStr.isEmpty())
	{
		return true;
	}
	unsigned int required = 0;
	if (!parseHostVersionString(minHostVersionStr, required))
	{
		return false;
	}
	return cloudsimPluginHostVersion() >= required;
}

bool PluginManager::aiSdkVersionSatisfies(const QString& minAiSdkVersionStr)
{
	if (minAiSdkVersionStr.isEmpty())
		return true;
	unsigned int required = 0;
	if (!parseHostVersionString(minAiSdkVersionStr, required))
		return false;
	return cloudsimAiSdkVersion() >= required;
}

namespace
{
bool manifestHasCapability(const nlohmann::json& manifest, const char* capability)
{
	if (!manifest.contains("capabilities") || !manifest["capabilities"].is_array())
		return false;
	for (const auto& c : manifest["capabilities"])
	{
		if (c.is_string() && c.get<std::string>() == capability)
			return true;
	}
	return false;
}
} // namespace

bool PluginManager::loadOnePlugin(const QString& pluginDir, const QString& manifestPath)
{
	QFile file(manifestPath);
	if (!file.open(QIODevice::ReadOnly))
	{
		RunLogger::warn(std::string("Plugin manifest not readable: ") + manifestPath.toStdString());
		return false;
	}

	nlohmann::json manifest;
	try
	{
		manifest = nlohmann::json::parse(file.readAll().constData());
	}
	catch (const std::exception& ex)
	{
		RunLogger::warn(std::string("Plugin manifest JSON error: ") + ex.what());
		return false;
	}

	if (!manifest.value("enabled", true))
	{
		return false;
	}

	const QString id = QString::fromStdString(manifest.value("id", std::string()));
	const QString name = QString::fromStdString(manifest.value("name", std::string()));
	const QString library = QString::fromStdString(manifest.value("library", std::string()));
	const QString minHost = QString::fromStdString(manifest.value("minHostVersion", std::string()));
	const QString minAiSdk = QString::fromStdString(manifest.value("minAiSdkVersion", std::string()));
	const bool wantsAi = manifestHasCapability(manifest, "ai-assistant");

	if (id.isEmpty() || library.isEmpty())
	{
		RunLogger::warn("Plugin manifest missing id or library: " + manifestPath.toStdString());
		return false;
	}

	if (!hostVersionSatisfies(minHost))
	{
		RunLogger::warn("Plugin skipped (host version): " + id.toStdString());
		return false;
	}

	const QString dllPath = QDir(pluginDir).absoluteFilePath(library);
	auto loader = std::make_unique<QPluginLoader>(dllPath);
	QObject* pluginObject = loader->instance();
	if (!pluginObject)
	{
		RunLogger::warn("Plugin load failed: " + id.toStdString() + " — " + loader->errorString().toStdString());
		return false;
	}

	auto* plugin = qobject_cast<ICloudSimPlugin*>(pluginObject);
	if (!plugin)
	{
		RunLogger::warn("Plugin does not implement ICloudSimPlugin: " + id.toStdString());
		loader->unload();
		return false;
	}

	if (plugin->pluginId() != id)
	{
		RunLogger::warn("Plugin id mismatch (manifest vs pluginId): " + id.toStdString());
		loader->unload();
		return false;
	}

	m_hostContext->beginPluginRegistration(id);
	if (!plugin->initialize(m_hostContext.get()))
	{
		m_hostContext->endPluginRegistration();
		RunLogger::warn("Plugin initialize() returned false: " + id.toStdString());
		loader->unload();
		return false;
	}

	if (wantsAi && !aiSdkVersionSatisfies(minAiSdk))
	{
		RunLogger::warn("Plugin AI capability skipped (AiSDK version): " + id.toStdString());
	}
	else if (IAiAssistantHost* aiHost = m_hostContext->aiAssistantHost())
	{
		if (auto* aiPlugin = qobject_cast<ICloudSimAiPlugin*>(pluginObject))
		{
			if (!aiPlugin->initializeAi(m_hostContext.get(), aiHost))
			{
				RunLogger::warn("Plugin initializeAi() returned false: " + id.toStdString());
			}
			else if (QWidget* parent = m_mainWindowHost->mainWindowWidget())
			{
				if (QWidget* panel = aiPlugin->createAssistantWidget(parent))
				{
					const int prio = aiPlugin->assistantPanelPriority();
					const QString tabTitle = aiPlugin->aiPluginId();
					m_hostContext->registerSidePanelTab(tabTitle.toUtf8().constData(), panel);
					(void)prio;
				}
			}
		}
	}
	m_hostContext->endPluginRegistration();

	auto loaded = std::make_unique<LoadedPlugin>();
	loaded->id = id;
	loaded->displayName = name.isEmpty() ? plugin->displayName() : name;
	loaded->loader = std::move(loader);
	loaded->instance = plugin;
	m_plugins.push_back(std::move(loaded));

	RunLogger::info("Plugin loaded: " + id.toStdString());
	return true;
}

void PluginManager::loadAllFromPluginsDirectory()
{
	if (!m_mainWindowHost || !m_hostContext)
	{
		return;
	}

	m_hostContext->attachDocumentTabSignals();
	m_hostContext->refreshDocumentAdapters();

	const QString pluginsRoot =
		QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("plugins"));
	QDir rootDir(pluginsRoot);
	if (!rootDir.exists())
	{
		m_loadSummary = QStringLiteral("Plugins: none (directory missing)");
		return;
	}

	int loadedCount = 0;
	int skippedCount = 0;
	const QStringList entries = rootDir.entryList(QDir::Dirs | QDir::NoDotAndDotDot);
	for (const QString& entry : entries)
	{
		const QString pluginDir = rootDir.absoluteFilePath(entry);
		const QString manifestPath = QDir(pluginDir).absoluteFilePath(QStringLiteral("plugin.json"));
		if (!QFile::exists(manifestPath))
		{
			continue;
		}
		if (loadOnePlugin(pluginDir, manifestPath))
		{
			++loadedCount;
		}
		else
		{
			++skippedCount;
		}
	}

	m_loadSummary = QStringLiteral("Plugins: %1 loaded, %2 skipped/failed").arg(loadedCount).arg(skippedCount);
	RunLogger::info(m_loadSummary.toStdString());

	if (m_mainWindowHost->statusBar())
	{
		m_mainWindowHost->statusBar()->showMessage(m_loadSummary, 8000);
	}
	m_mainWindowHost->appendRunInfo(m_loadSummary);
}

void PluginManager::notifyLanguageChanged()
{
	if (m_hostContext)
	{
		m_hostContext->notifyLanguageChanged();
	}
}

void PluginManager::invokeProjectAboutToSave(const QString& documentId, QJsonObject& root)
{
	if (m_hostContext)
	{
		m_hostContext->invokeProjectAboutToSave(documentId, root);
	}
}

void PluginManager::invokeProjectLoaded(const QString& documentId, const QJsonObject& root)
{
	if (m_hostContext)
	{
		m_hostContext->invokeProjectLoaded(documentId, root);
	}
}
