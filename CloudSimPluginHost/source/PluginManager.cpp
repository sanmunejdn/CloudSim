#include "PluginManager.h"

#include "CloudSimPluginVersion.h"
#include "ICloudSimPlugin.h"
#include "MainWindow.h"
#include "PluginHostContext.h"
#include "RunLogger.h"

#include <json.hpp>

#include <QCoreApplication>
#include <QDir>
#include <QFile>
#include <QPluginLoader>
#include <QStatusBar>

PluginManager::PluginManager(MainWindow* mainWindow, QObject* parent)
	: QObject(parent)
	, m_mainWindow(mainWindow)
	, m_hostContext(std::make_unique<PluginHostContext>(mainWindow, this))
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
	for (auto& entry : m_plugins)
	{
		if (entry && entry->instance)
		{
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

	if (!plugin->initialize(m_hostContext.get()))
	{
		RunLogger::warn("Plugin initialize() returned false: " + id.toStdString());
		loader->unload();
		return false;
	}

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
	if (!m_mainWindow || !m_hostContext)
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

	if (m_mainWindow->statusBar())
	{
		m_mainWindow->statusBar()->showMessage(m_loadSummary, 8000);
	}
	if (m_mainWindow->runInfoPage())
	{
		m_mainWindow->runInfoPage()->appendInfo(m_loadSummary);
	}
}
