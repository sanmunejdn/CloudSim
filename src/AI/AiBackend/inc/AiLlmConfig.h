#pragma once

#include "aibackend_global.h"

#include <QString>
#include <optional>

/// Loaded from `ai_config.json` next to the executable (see defaultAiConfigPath).
struct AIBACKEND_EXPORT AiLlmConfig
{
	bool enabled = false;
	/// When true, try AiIntentParser before calling the LLM.
	bool ruleParserFirst = false;
	QString baseUrl = QStringLiteral("https://api.openai.com/v1");
	QString apiKey;
	/// If api_key is empty, read from this environment variable.
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
