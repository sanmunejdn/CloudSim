#include "Ai/AiConfigLoader.h"

#include "Ai/AiMeshDefaults.h"
#include "AiConfigDefaults.h"
#include "AiDomainTypes.h"
#include "AiLlmConfig.h"

#include <json.hpp>

#include <QCoreApplication>
#include <QDir>
#include <QFile>

#include <algorithm>

static QStringList jsonToStringList(const nlohmann::json& arr)
{
	QStringList out;
	if (!arr.is_array())
		return out;
	for (const auto& v : arr)
	{
		if (v.is_string())
			out.push_back(QString::fromStdString(v.get<std::string>()));
	}
	return out;
}

std::optional<AiConfigDto> loadAiConfigDto(const QString& filePath)
{
	const QString path = filePath.isEmpty() ? defaultAiConfigPath() : filePath;
	QFile f(path);
	if (!f.open(QIODevice::ReadOnly))
	{
		const QString defaultsPath = QCoreApplication::applicationDirPath() + QStringLiteral("/ai_config.defaults.json");
		QFile def(defaultsPath);
		if (def.open(QIODevice::ReadOnly))
		{
			try
			{
				const nlohmann::json dj = nlohmann::json::parse(def.readAll().constData(), nullptr, true);
				if (dj.is_object())
					AiMeshDefaults::loadFromConfigJson(dj);
			}
			catch (...)
			{
			}
		}
		return std::nullopt;
	}

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

	AiMeshDefaults::loadFromConfigJson(j);

	AiConfigDto cfg = defaultAiConfigDto();

	if (j.contains("hardware_profile") && j["hardware_profile"].is_string())
		cfg.hardwareProfile = QString::fromStdString(j["hardware_profile"].get<std::string>());
	if (j.contains("parser_priority"))
	{
		const QStringList pl = jsonToStringList(j["parser_priority"]);
		if (!pl.isEmpty())
			cfg.parserPriorityDefault = pl;
	}

	if (j.contains("remote_llm") && j["remote_llm"].is_object())
	{
		const auto& r = j["remote_llm"];
		cfg.remoteLlm.enabled = r.value("enabled", false);
		if (r.contains("base_url"))
			cfg.remoteLlm.baseUrl = QString::fromStdString(r["base_url"].get<std::string>());
		if (r.contains("api_key"))
			cfg.remoteLlm.apiKey = QString::fromStdString(r["api_key"].get<std::string>());
		if (r.contains("api_key_env"))
			cfg.remoteLlm.apiKeyEnv = QString::fromStdString(r["api_key_env"].get<std::string>());
		if (r.contains("model"))
			cfg.remoteLlm.model = QString::fromStdString(r["model"].get<std::string>());
		cfg.remoteLlm.timeoutMs = std::max(5000, r.value("timeout_ms", cfg.remoteLlm.timeoutMs));
		cfg.remoteLlm.temperature = r.value("temperature", cfg.remoteLlm.temperature);
	}

	// 兼容旧 ai_config：enabled / rule_parser_first / base_url / model
	if (j.contains("enabled") || j.contains("base_url") || j.contains("model"))
	{
		cfg.remoteLlm.enabled = j.value("enabled", cfg.remoteLlm.enabled);
		if (j.contains("base_url"))
			cfg.remoteLlm.baseUrl = QString::fromStdString(j["base_url"].get<std::string>()).trimmed();
		if (j.contains("api_key"))
			cfg.remoteLlm.apiKey = QString::fromStdString(j["api_key"].get<std::string>()).trimmed();
		if (j.contains("api_key_env"))
			cfg.remoteLlm.apiKeyEnv = QString::fromStdString(j["api_key_env"].get<std::string>()).trimmed();
		if (j.contains("model"))
			cfg.remoteLlm.model = QString::fromStdString(j["model"].get<std::string>()).trimmed();
		cfg.remoteLlm.timeoutMs = std::max(5000, j.value("timeout_ms", cfg.remoteLlm.timeoutMs));
		cfg.remoteLlm.temperature = j.value("temperature", cfg.remoteLlm.temperature);
		const bool ruleFirst = j.value("rule_parser_first", false);
		if (ruleFirst && !cfg.domains.empty())
			cfg.domains[0].parserPriority = QStringList{
				QStringLiteral("rules"),
				QStringLiteral("local"),
				QStringLiteral("remote")
			};
	}

	if (j.contains("router") && j["router"].is_object())
	{
		const auto& r = j["router"];
		if (r.contains("mode"))
			cfg.router.mode = QString::fromStdString(r["mode"].get<std::string>());
		if (r.contains("local_model"))
			cfg.router.localModel = QString::fromStdString(r["local_model"].get<std::string>());
		if (r.contains("base_url"))
			cfg.router.baseUrl = QString::fromStdString(r["base_url"].get<std::string>());
	}

	if (j.contains("domains") && j["domains"].is_array())
	{
		cfg.domains.clear();
		for (const auto& d : j["domains"])
		{
			if (!d.is_object())
				continue;
			AiDomainModelConfig dm;
			dm.id = QString::fromStdString(d.value("id", std::string()));
			dm.enabled = d.value("enabled", true);
			if (d.contains("base_url"))
				dm.baseUrl = QString::fromStdString(d["base_url"].get<std::string>());
			if (d.contains("model"))
				dm.model = QString::fromStdString(d["model"].get<std::string>());
			dm.multimodal = d.value("multimodal", false);
			if (d.contains("parser_priority"))
				dm.parserPriority = jsonToStringList(d["parser_priority"]);
			dm.unloadOtherModelsBeforeInfer = d.value("unload_other_models_before_infer", false);
			if (!dm.id.isEmpty())
				cfg.domains.push_back(dm);
		}
	}

	return cfg;
}

bool saveAiConfigDto(const AiConfigDto& config, const QString& filePath, QString* errorMessage)
{
	nlohmann::json j;
	j["hardware_profile"] = config.hardwareProfile.toStdString();
	j["parser_priority"] = nlohmann::json::array();
	for (const QString& s : config.parserPriorityDefault)
		j["parser_priority"].push_back(s.toStdString());

	j["remote_llm"] = {
		{ "enabled", config.remoteLlm.enabled },
		{ "base_url", config.remoteLlm.baseUrl.toStdString() },
		{ "api_key", config.remoteLlm.apiKey.toStdString() },
		{ "api_key_env", config.remoteLlm.apiKeyEnv.toStdString() },
		{ "model", config.remoteLlm.model.toStdString() },
		{ "timeout_ms", config.remoteLlm.timeoutMs },
		{ "temperature", config.remoteLlm.temperature }
	};
	j["router"] = {
		{ "mode", config.router.mode.toStdString() },
		{ "local_model", config.router.localModel.toStdString() },
		{ "base_url", config.router.baseUrl.toStdString() }
	};
	j["domains"] = nlohmann::json::array();
	for (const AiDomainModelConfig& d : config.domains)
	{
		nlohmann::json o;
		o["id"] = d.id.toStdString();
		o["enabled"] = d.enabled;
		o["base_url"] = d.baseUrl.toStdString();
		o["model"] = d.model.toStdString();
		o["multimodal"] = d.multimodal;
		o["unload_other_models_before_infer"] = d.unloadOtherModelsBeforeInfer;
		o["parser_priority"] = nlohmann::json::array();
		for (const QString& p : d.parserPriority)
			o["parser_priority"].push_back(p.toStdString());
		j["domains"].push_back(std::move(o));
	}

	const QString path = filePath.isEmpty() ? defaultAiConfigPath() : filePath;
	QFile f(path);
	if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		if (errorMessage)
			*errorMessage = QStringLiteral("Cannot write %1").arg(path);
		return false;
	}
	const std::string dumped = j.dump(2);
	f.write(dumped.data(), static_cast<qint64>(dumped.size()));
	return true;
}
