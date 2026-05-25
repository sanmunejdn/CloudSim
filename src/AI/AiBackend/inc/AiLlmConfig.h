#pragma once

#include "aibackend_global.h"

#include <QString>
#include <optional>

/// 从可执行文件旁 ai_config.json 加载
struct AIBACKEND_EXPORT AiLlmConfig
{
	bool enabled = false;
	/// true 时先走 AiIntentParser 再调 LLM
	bool ruleParserFirst = false;
	QString baseUrl = QStringLiteral("https://api.openai.com/v1");
	QString apiKey;
	/// api_key 空时从此环境变量读取
	QString apiKeyEnv = QStringLiteral("OPENAI_API_KEY");
	QString model = QStringLiteral("gpt-4o-mini");
	int timeoutMs = 60000;
	double temperature = 0.1;

	bool hasApiKey() const;
};

AIBACKEND_EXPORT QString defaultAiConfigPath();
AIBACKEND_EXPORT AiLlmConfig defaultAiLlmConfig();
AIBACKEND_EXPORT std::optional<AiLlmConfig> loadAiLlmConfig(const QString& filePath = QString());
AIBACKEND_EXPORT bool saveAiLlmConfig(
	const AiLlmConfig& config,
	const QString& filePath = QString(),
	QString* errorMessage = nullptr);
