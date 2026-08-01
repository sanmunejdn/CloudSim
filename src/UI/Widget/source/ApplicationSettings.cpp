/// @file ApplicationSettings.cpp
/// @brief ApplicationSettings 实现

#include "ApplicationSettings.h"

#include "RunLogger.h"

#include <QCoreApplication>
#include <QDir>
#include <QFileInfo>
#include <QSaveFile>
#include <QSettings>
#include <QString>
#include <QStandardPaths>
#include <QTextStream>
#include <QWidget>

namespace
{
constexpr int kSettingsSchemaVersion = 1;

static const QString kSettingsFileName = QStringLiteral("settings.ini");

QString decodeSidePanelTabStorageKey(const QString& storedKey)
{
	// 旧版写入时把 '.' 换成 '_'，需还原为 widget objectName
	if (storedKey.startsWith(QStringLiteral("CloudSim_PluginTab_")))
	{
		return QString(storedKey).replace(QLatin1Char('_'), QLatin1Char('.'));
	}
	return storedKey;
}

bool isEphemeralSidePanelTabKey(const QString& key)
{
	// 旧版用指针地址作键，重启后失效
	return key.startsWith(QStringLiteral("CloudSim.SidePanel."));
}

ApplicationStyle::Theme themeFromString(const QString& value)
{
	return value.compare(QStringLiteral("dark"), Qt::CaseInsensitive) == 0 ? ApplicationStyle::Theme::Dark
																			: ApplicationStyle::Theme::Light;
}

QString themeToString(const ApplicationStyle::Theme theme)
{
	return theme == ApplicationStyle::Theme::Dark ? QStringLiteral("dark") : QStringLiteral("light");
}

QString legacyAppConfigSettingsPath()
{
	const QString configDir = QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
	if (configDir.isEmpty())
	{
		return {};
	}
	return QDir(configDir).absoluteFilePath(kSettingsFileName);
}

ApplicationSettings::UiPreferences readFromSettingsFile(const QString& path)
{
	ApplicationSettings::UiPreferences prefs;
	QSettings storage(path, QSettings::IniFormat);

	storage.beginGroup(QStringLiteral("General"));
	prefs.language = storage.value(QStringLiteral("language"), QStringLiteral("zh")).toString();
	storage.endGroup();

	storage.beginGroup(QStringLiteral("Appearance"));
	prefs.theme = themeFromString(storage.value(QStringLiteral("theme"), QStringLiteral("light")).toString());
	storage.endGroup();

	storage.beginGroup(QStringLiteral("View"));
	prefs.leftPanelVisible = storage.value(QStringLiteral("leftPanelVisible"), true).toBool();
	prefs.rightPanelVisible = storage.value(QStringLiteral("rightPanelVisible"), true).toBool();
	prefs.leftDockWidth = storage.value(QStringLiteral("leftDockWidth"), 240).toInt();
	prefs.rightDockWidth = storage.value(QStringLiteral("rightDockWidth"), 360).toInt();
	storage.endGroup();

	storage.beginGroup(QStringLiteral("SidePanelTabs"));
	for (const QString& storedKey : storage.childKeys())
	{
		const QString logicalKey = decodeSidePanelTabStorageKey(storedKey);
		if (!isEphemeralSidePanelTabKey(logicalKey))
		{
			prefs.sidePanelTabs.insert(logicalKey, storage.value(storedKey, true).toBool());
		}
	}
	for (const QString& group : storage.childGroups())
	{
		storage.beginGroup(group);
		for (const QString& nestedKey : storage.childKeys())
		{
			const QString logicalKey = decodeSidePanelTabStorageKey(group + QLatin1Char('.') + nestedKey);
			if (!isEphemeralSidePanelTabKey(logicalKey))
			{
				prefs.sidePanelTabs.insert(logicalKey, storage.value(nestedKey, true).toBool());
			}
		}
		storage.endGroup();
	}
	storage.endGroup();

	storage.beginGroup(QStringLiteral("Workspace"));
	prefs.workspaceModeId = storage.value(QStringLiteral("modeId")).toString();
	storage.endGroup();

	return prefs;
}

/// 旧版主题写在系统注册表，首次读 ini 时迁移一次
ApplicationStyle::Theme loadLegacyTheme()
{
	QSettings legacy;
	legacy.beginGroup(QStringLiteral("Appearance"));
	const QString value = legacy.value(QStringLiteral("theme"), QStringLiteral("light")).toString();
	legacy.endGroup();
	return themeFromString(value);
}

void migrateLegacyThemeIfNeeded(ApplicationSettings::UiPreferences& prefs)
{
	if (prefs.theme != ApplicationStyle::Theme::Light)
	{
		return;
	}
	QSettings legacy;
	legacy.beginGroup(QStringLiteral("Appearance"));
	if (legacy.contains(QStringLiteral("theme")))
	{
		prefs.theme = themeFromString(legacy.value(QStringLiteral("theme")).toString());
	}
	legacy.endGroup();
}

QString boolIniValue(const bool value)
{
	return value ? QStringLiteral("true") : QStringLiteral("false");
}

/// QSettings::sync 在部分环境下不落盘，改与 ai_config.json 相同的 QFile 写入
bool writePreferencesIni(const QString& path, const ApplicationSettings::UiPreferences& prefs)
{
	QSaveFile file(path);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Text))
	{
		RunLogger::warn(std::string("ApplicationSettings: cannot open for write: ") + path.toStdString());
		return false;
	}

	QTextStream out(&file);
	out.setCodec("UTF-8");
	out << "[General]\n";
	out << "schemaVersion=" << kSettingsSchemaVersion << '\n';
	out << "language=" << prefs.language << "\n\n";

	out << "[Appearance]\n";
	out << "theme=" << themeToString(prefs.theme) << "\n\n";

	out << "[View]\n";
	out << "leftPanelVisible=" << boolIniValue(prefs.leftPanelVisible) << '\n';
	out << "rightPanelVisible=" << boolIniValue(prefs.rightPanelVisible) << '\n';
	out << "leftDockWidth=" << prefs.leftDockWidth << '\n';
	out << "rightDockWidth=" << prefs.rightDockWidth << "\n\n";

	out << "[SidePanelTabs]\n";
	for (auto it = prefs.sidePanelTabs.constBegin(); it != prefs.sidePanelTabs.constEnd(); ++it)
	{
		if (!it.key().isEmpty() && !isEphemeralSidePanelTabKey(it.key()))
		{
			out << it.key() << '=' << boolIniValue(it.value()) << '\n';
		}
	}
	out << '\n';

	out << "[Workspace]\n";
	if (!prefs.workspaceModeId.isEmpty())
	{
		out << "modeId=" << prefs.workspaceModeId << '\n';
	}

	if (!file.commit())
	{
		RunLogger::warn(std::string("ApplicationSettings: commit failed: ") + path.toStdString());
		return false;
	}
	return true;
}

} // namespace

namespace ApplicationSettings
{
QString settingsFilePath()
{
	if (!QCoreApplication::instance())
	{
		return {};
	}
	// 与 ai_config.json 相同：放在 exe 旁，VS 调试/发布目录均稳定可写
	return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(kSettingsFileName);
}

QString sidePanelTabKey(const QWidget* widget)
{
	if (!widget)
	{
		return {};
	}
	if (!widget->objectName().isEmpty())
	{
		return widget->objectName();
	}
	return QStringLiteral("CloudSim.SidePanel.") + QString::number(reinterpret_cast<quintptr>(widget), 16);
}

bool sidePanelTabVisible(const QString& key, const bool defaultVisible)
{
	if (key.isEmpty())
	{
		return defaultVisible;
	}
	return load().sidePanelTabs.value(key, defaultVisible);
}

UiPreferences load()
{
	const QString primaryPath = settingsFilePath();
	if (!primaryPath.isEmpty() && QFileInfo::exists(primaryPath))
	{
		return readFromSettingsFile(primaryPath);
	}

	const QString legacyPath = legacyAppConfigSettingsPath();
	if (!legacyPath.isEmpty() && QFileInfo::exists(legacyPath))
	{
		UiPreferences prefs = readFromSettingsFile(legacyPath);
		save(prefs);
		return prefs;
	}

	UiPreferences prefs;
	migrateLegacyThemeIfNeeded(prefs);
	return prefs;
}

void save(const UiPreferences& prefs)
{
	const QString path = QDir::fromNativeSeparators(settingsFilePath());
	if (path.isEmpty())
	{
		RunLogger::warn("ApplicationSettings: settings path unavailable, preferences not saved.");
		return;
	}

	const QFileInfo fileInfo(path);
	if (!fileInfo.dir().exists() && !QDir().mkpath(fileInfo.absolutePath()))
	{
		RunLogger::warn(std::string("ApplicationSettings: cannot create directory for ") + path.toStdString());
		return;
	}

	if (!writePreferencesIni(path, prefs))
	{
		return;
	}
}

void saveTheme(const ApplicationStyle::Theme theme)
{
	UiPreferences prefs = load();
	prefs.theme = theme;
	save(prefs);
}

ApplicationStyle::Theme loadTheme()
{
	return load().theme;
}

} // namespace ApplicationSettings
