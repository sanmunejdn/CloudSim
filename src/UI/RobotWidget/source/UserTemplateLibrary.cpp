/// @file UserTemplateLibrary.cpp
/// @brief 用户命名模板库实现

#include "UserTemplateLibrary.h"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QSettings>
#include <QStandardPaths>
#include <QUuid>

namespace
{
QString kindFolderName(UserTemplateKind kind)
{
	return kind == UserTemplateKind::Pipeline ? QStringLiteral("pipeline") : QStringLiteral("discretize");
}

QString indexPath(UserTemplateKind kind)
{
	return UserTemplateLibrary::templatesRoot(kind) + QStringLiteral("/index.json");
}

QString entryPath(UserTemplateKind kind, const QString& id)
{
	return UserTemplateLibrary::templatesRoot(kind) + QLatin1Char('/') + id + QStringLiteral(".json");
}

bool ensureDir(UserTemplateKind kind, QString* outError)
{
	const QString root = UserTemplateLibrary::templatesRoot(kind);
	if (QDir().mkpath(root))
	{
		return true;
	}
	if (outError)
	{
		*outError = QStringLiteral("无法创建模板目录: %1").arg(root);
	}
	return false;
}

nlohmann::json readJsonFile(const QString& path, QString* outError)
{
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
	{
		if (outError)
		{
			*outError = QStringLiteral("无法读取: %1").arg(path);
		}
		return nlohmann::json();
	}
	const QByteArray bytes = f.readAll();
	nlohmann::json j = nlohmann::json::parse(bytes.constData(), nullptr, false);
	if (j.is_discarded())
	{
		if (outError)
		{
			*outError = QStringLiteral("JSON 无效: %1").arg(path);
		}
		return nlohmann::json();
	}
	return j;
}

bool writeJsonFile(const QString& path, const nlohmann::json& j, QString* outError)
{
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		if (outError)
		{
			*outError = QStringLiteral("无法写入: %1").arg(path);
		}
		return false;
	}
	const std::string text = j.dump(2);
	if (f.write(text.data(), static_cast<qint64>(text.size())) < 0)
	{
		if (outError)
		{
			*outError = QStringLiteral("写入失败: %1").arg(path);
		}
		return false;
	}
	return true;
}

nlohmann::json loadIndex(UserTemplateKind kind)
{
	const QString path = indexPath(kind);
	if (!QFileInfo::exists(path))
	{
		return nlohmann::json::object({{"version", 1}, {"entries", nlohmann::json::array()}});
	}
	QString err;
	nlohmann::json j = readJsonFile(path, &err);
	if (j.is_null() || !j.is_object())
	{
		return nlohmann::json::object({{"version", 1}, {"entries", nlohmann::json::array()}});
	}
	if (!j.contains("entries") || !j["entries"].is_array())
	{
		j["entries"] = nlohmann::json::array();
	}
	return j;
}

bool saveIndex(UserTemplateKind kind, const nlohmann::json& index, QString* outError)
{
	return writeJsonFile(indexPath(kind), index, outError);
}

QString findIdByName(const nlohmann::json& index, const QString& name)
{
	if (!index.contains("entries") || !index["entries"].is_array())
	{
		return {};
	}
	const std::string nameUtf8 = name.toStdString();
	for (const auto& e : index["entries"])
	{
		if (e.is_object() && e.value("name", std::string()) == nameUtf8)
		{
			return QString::fromStdString(e.value("id", std::string()));
		}
	}
	return {};
}
} // namespace

QString UserTemplateLibrary::templatesRoot(UserTemplateKind kind)
{
	const QString base = QStandardPaths::writableLocation(QStandardPaths::AppDataLocation);
	return base + QStringLiteral("/CloudSim/templates/") + kindFolderName(kind);
}

QVector<UserTemplateEntry> UserTemplateLibrary::list(UserTemplateKind kind)
{
	QVector<UserTemplateEntry> out;
	const nlohmann::json index = loadIndex(kind);
	for (const auto& e : index["entries"])
	{
		if (!e.is_object())
		{
			continue;
		}
		UserTemplateEntry entry;
		entry.id = QString::fromStdString(e.value("id", std::string()));
		entry.name = QString::fromStdString(e.value("name", std::string()));
		entry.updatedAtIso = QString::fromStdString(e.value("updatedAt", std::string()));
		if (!entry.id.isEmpty())
		{
			out.push_back(entry);
		}
	}
	return out;
}

bool UserTemplateLibrary::save(UserTemplateKind kind, const QString& name, const nlohmann::json& payload, QString* outId,
							   QString* outError)
{
	const QString trimmed = name.trimmed();
	if (trimmed.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("模板名称不能为空");
		}
		return false;
	}
	if (!ensureDir(kind, outError))
	{
		return false;
	}

	nlohmann::json index = loadIndex(kind);
	QString id = findIdByName(index, trimmed);
	if (id.isEmpty())
	{
		id = QUuid::createUuid().toString(QUuid::WithoutBraces);
	}

	const QString now = QDateTime::currentDateTimeUtc().toString(Qt::ISODate);
	nlohmann::json file = nlohmann::json::object();
	file["version"] = 1;
	file["id"] = id.toStdString();
	file["name"] = trimmed.toStdString();
	file["updatedAt"] = now.toStdString();
	if (kind == UserTemplateKind::Pipeline)
	{
		if (payload.is_array())
		{
			file["pipeline"] = payload;
		}
		else if (payload.is_object() && payload.contains("pipeline"))
		{
			file["pipeline"] = payload["pipeline"];
		}
		else
		{
			if (outError)
			{
				*outError = QStringLiteral("流水线模板 payload 无效");
			}
			return false;
		}
	}
	else
	{
		if (!payload.is_object() || !payload.contains("strategyId"))
		{
			if (outError)
			{
				*outError = QStringLiteral("离散模板需含 strategyId");
			}
			return false;
		}
		file["strategyId"] = payload["strategyId"];
		file["params"] = payload.contains("params") ? payload["params"] : nlohmann::json::object();
	}

	if (!writeJsonFile(entryPath(kind, id), file, outError))
	{
		return false;
	}

	bool found = false;
	for (auto& e : index["entries"])
	{
		if (e.is_object() && QString::fromStdString(e.value("id", std::string())) == id)
		{
			e["name"] = trimmed.toStdString();
			e["updatedAt"] = now.toStdString();
			found = true;
			break;
		}
	}
	if (!found)
	{
		index["entries"].push_back(nlohmann::json::object({{"id", id.toStdString()},
														   {"name", trimmed.toStdString()},
														   {"updatedAt", now.toStdString()}}));
	}
	if (!saveIndex(kind, index, outError))
	{
		return false;
	}
	if (outId)
	{
		*outId = id;
	}
	return true;
}

bool UserTemplateLibrary::load(UserTemplateKind kind, const QString& id, nlohmann::json* outPayload, QString* outName,
							   QString* outError)
{
	if (!outPayload || id.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("参数无效");
		}
		return false;
	}
	nlohmann::json file = readJsonFile(entryPath(kind, id), outError);
	if (file.is_null() || !file.is_object())
	{
		return false;
	}
	if (outName)
	{
		*outName = QString::fromStdString(file.value("name", std::string()));
	}
	if (kind == UserTemplateKind::Pipeline)
	{
		if (!file.contains("pipeline"))
		{
			if (outError)
			{
				*outError = QStringLiteral("缺少 pipeline 字段");
			}
			return false;
		}
		*outPayload = file["pipeline"];
	}
	else
	{
		nlohmann::json payload = nlohmann::json::object();
		payload["strategyId"] = file.value("strategyId", std::string());
		payload["params"] = file.contains("params") ? file["params"] : nlohmann::json::object();
		*outPayload = std::move(payload);
	}
	return true;
}

bool UserTemplateLibrary::remove(UserTemplateKind kind, const QString& id, QString* outError)
{
	if (id.isEmpty())
	{
		if (outError)
		{
			*outError = QStringLiteral("未选择模板");
		}
		return false;
	}
	nlohmann::json index = loadIndex(kind);
	nlohmann::json next = nlohmann::json::array();
	for (const auto& e : index["entries"])
	{
		if (!e.is_object() || QString::fromStdString(e.value("id", std::string())) != id)
		{
			next.push_back(e);
		}
	}
	index["entries"] = std::move(next);
	if (!saveIndex(kind, index, outError))
	{
		return false;
	}
	QFile::remove(entryPath(kind, id));
	return true;
}

bool UserTemplateLibrary::importFile(UserTemplateKind kind, const QString& filePath, QString* outId, QString* outError)
{
	nlohmann::json file = readJsonFile(filePath, outError);
	if (file.is_null() || !file.is_object())
	{
		return false;
	}
	QString name = QString::fromStdString(file.value("name", std::string()));
	if (name.isEmpty())
	{
		name = QFileInfo(filePath).completeBaseName();
	}
	nlohmann::json payload;
	if (kind == UserTemplateKind::Pipeline)
	{
		if (file.contains("pipeline"))
		{
			payload = file["pipeline"];
		}
		else if (file.is_array())
		{
			payload = file;
		}
		else
		{
			if (outError)
			{
				*outError = QStringLiteral("导入文件缺少 pipeline");
			}
			return false;
		}
	}
	else
	{
		if (!file.contains("strategyId"))
		{
			if (outError)
			{
				*outError = QStringLiteral("导入文件缺少 strategyId");
			}
			return false;
		}
		payload = nlohmann::json::object({{"strategyId", file["strategyId"]},
										  {"params", file.contains("params") ? file["params"] : nlohmann::json::object()}});
	}
	return save(kind, name, payload, outId, outError);
}

bool UserTemplateLibrary::exportFile(UserTemplateKind kind, const QString& id, const QString& filePath,
									 QString* outError)
{
	nlohmann::json payload;
	QString name;
	if (!load(kind, id, &payload, &name, outError))
	{
		return false;
	}
	nlohmann::json file = nlohmann::json::object();
	file["version"] = 1;
	file["id"] = id.toStdString();
	file["name"] = name.toStdString();
	file["updatedAt"] = QDateTime::currentDateTimeUtc().toString(Qt::ISODate).toStdString();
	if (kind == UserTemplateKind::Pipeline)
	{
		file["pipeline"] = payload;
	}
	else
	{
		file["strategyId"] = payload.value("strategyId", std::string());
		file["params"] = payload.contains("params") ? payload["params"] : nlohmann::json::object();
	}
	return writeJsonFile(filePath, file, outError);
}

void UserTemplateLibrary::migrateLegacyPipelineSlot()
{
	QSettings settings(QStringLiteral("CloudSim"), QStringLiteral("TrajectoryPipeline"));
	if (settings.value(QStringLiteral("pipelineJsonMigrated"), false).toBool())
	{
		return;
	}
	const QString jsonText = settings.value(QStringLiteral("pipelineJson")).toString();
	if (jsonText.isEmpty())
	{
		settings.setValue(QStringLiteral("pipelineJsonMigrated"), true);
		return;
	}
	nlohmann::json pipelineJson = nlohmann::json::parse(jsonText.toStdString(), nullptr, false);
	if (pipelineJson.is_discarded() || !pipelineJson.is_array())
	{
		settings.setValue(QStringLiteral("pipelineJsonMigrated"), true);
		return;
	}
	QString err;
	(void)save(UserTemplateKind::Pipeline, QStringLiteral("迁移的上次保存"), pipelineJson, nullptr, &err);
	settings.setValue(QStringLiteral("pipelineJsonMigrated"), true);
}
