#ifndef CLOUDSIMAISDK_AICONFIGDTO_H
#define CLOUDSIMAISDK_AICONFIGDTO_H

/// @file AiConfigDto.h
/// @brief AiConfigDto 接口

#include "cloudsim_ai_sdk_global.h"

#include <QString>
#include <QStringList>
#include <vector>

struct CLOUDSIM_AI_SDK_EXPORT AiRemoteLlmConfig
{
	bool enabled = false;
	QString baseUrl = QStringLiteral("https://api.openai.com/v1");
	QString apiKey;
	QString apiKeyEnv = QStringLiteral("OPENAI_API_KEY");
	QString model = QStringLiteral("gpt-4o-mini");
	int timeoutMs = 60000;
	double temperature = 0.1;
};

struct CLOUDSIM_AI_SDK_EXPORT AiDomainModelConfig
{
	QString id;
	bool enabled = true;
	QString baseUrl = QStringLiteral("http://127.0.0.1:11434/v1");
	QString model = QStringLiteral("qwen2.5:3b");
	bool multimodal = false;
	QStringList parserPriority;
	bool unloadOtherModelsBeforeInfer = false;
};

struct CLOUDSIM_AI_SDK_EXPORT AiRouterConfig
{
	QString mode = QStringLiteral("explicit_ui");
	QString localModel;
	QString baseUrl = QStringLiteral("http://127.0.0.1:11434/v1");
};

struct CLOUDSIM_AI_SDK_EXPORT AiAgentPolicyConfig
{
	int maxSteps = 8;
	bool autoExecuteLowRisk = true;
	bool enableTrace = true;
	bool enablePlan = true;
	int planMaxSteps = 8;
	bool replanOnFailure = true;
};

struct CLOUDSIM_AI_SDK_EXPORT AiConfigDto
{
	QString hardwareProfile = QStringLiteral("vram_8gb");
	QStringList parserPriorityDefault = {QStringLiteral("rules"), QStringLiteral("local"), QStringLiteral("remote")};
	AiRemoteLlmConfig remoteLlm;
	AiRouterConfig router;
	AiAgentPolicyConfig agent;
	std::vector<AiDomainModelConfig> domains;
};

#endif // CLOUDSIMAISDK_AICONFIGDTO_H
