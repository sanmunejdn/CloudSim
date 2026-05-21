#include "AiLlmConfig.h"

#include <json.hpp>

#include <QCoreApplication>
#include <QDir>
#include <QFile>

#include <algorithm>

bool AiLlmConfig::hasApiKey() const
{
	if (!apiKey.trimmed().isEmpty())
		return true;
	if (!apiKeyEnv.trimmed().isEmpty())
	{
		const QByteArray v = qgetenv(apiKeyEnv.toUtf8().constData());
		return !v.trimmed().isEmpty();
	}
	return false;
}

QString defaultAiConfigPath()
{
	return QDir(QCoreApplication::applicationDirPath()).absoluteFilePath(QStringLiteral("ai_config.json"));
}

AiLlmConfig defaultAiLlmConfig()
{
	return AiLlmConfig{};
}

std::optional<AiLlmConfig> loadAiLlmConfig(const QString& filePath)
{
	const QString path = filePath.isEmpty() ? defaultAiConfigPath() : filePath;
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
		return std::nullopt;

	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(f.readAll().constData(), nullptr, true);
	}
	catch (...)
	{
		return std::nullopt;
	}
	if (!j.is_object())
		return std::nullopt;

	AiLlmConfig cfg;
	cfg.enabled = j.value("enabled", false);
	cfg.ruleParserFirst = j.value("rule_parser_first", false);
	if (j.contains("base_url") && j["base_url"].is_string())
		cfg.baseUrl = QString::fromStdString(j["base_url"].get<std::string>()).trimmed();
	if (j.contains("api_key") && j["api_key"].is_string())
		cfg.apiKey = QString::fromStdString(j["api_key"].get<std::string>()).trimmed();
	if (j.contains("api_key_env") && j["api_key_env"].is_string())
		cfg.apiKeyEnv = QString::fromStdString(j["api_key_env"].get<std::string>()).trimmed();
	if (j.contains("model") && j["model"].is_string())
		cfg.model = QString::fromStdString(j["model"].get<std::string>()).trimmed();
	cfg.timeoutMs = std::max(5000, j.value("timeout_ms", cfg.timeoutMs));
	cfg.temperature = j.value("temperature", cfg.temperature);
	return cfg;
}

bool saveAiLlmConfig(const AiLlmConfig& config, const QString& filePath, QString* errorMessage)
{
	const QString path = filePath.isEmpty() ? defaultAiConfigPath() : filePath;
	nlohmann::json j;
	j["enabled"] = config.enabled;
	j["rule_parser_first"] = config.ruleParserFirst;
	j["base_url"] = config.baseUrl.trimmed().toStdString();
	j["api_key"] = config.apiKey.toStdString();
	j["api_key_env"] = config.apiKeyEnv.trimmed().toStdString();
	j["model"] = config.model.trimmed().toStdString();
	j["timeout_ms"] = std::max(5000, config.timeoutMs);
	j["temperature"] = config.temperature;

	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("Cannot write %1").arg(path);
		return false;
	}
	const std::string dumped = j.dump(2);
	if (f.write(dumped.data(), static_cast<qint64>(dumped.size())) != static_cast<qint64>(dumped.size()))
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("Failed to write %1").arg(path);
		return false;
	}
	return true;
}
