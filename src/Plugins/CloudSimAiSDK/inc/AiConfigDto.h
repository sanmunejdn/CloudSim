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
	/// explicit_ui | rules_score | local_classify
	QString mode = QStringLiteral("rules_score");
	QString localModel;
	QString baseUrl = QStringLiteral("http://127.0.0.1:11434/v1");
	/// rules_score 最低接受分；local_classify 在规则无域时再用小模型选域
	int minScore = 2;
};

struct CLOUDSIM_AI_SDK_EXPORT AiAgentPolicyConfig
{
	int maxSteps = 8;
	/// 默认关闭：低风险空 schema 也不自动执行，一律确认
	bool autoExecuteLowRisk = false;
	bool enableTrace = true;
	bool enablePlan = true;
	int planMaxSteps = 8;
	bool replanOnFailure = true;
	/// 无 Catalog 关键词可靠命中时，不调用 LLM tool_calls，强制澄清
	bool requireKeywordHit = true;
};

struct CLOUDSIM_AI_SDK_EXPORT AiConfigDto
{
	QString hardwareProfile = QStringLiteral("vram_8gb");
	QStringList parserPriorityDefault = {QStringLiteral("rules"), QStringLiteral("local"), QStringLiteral("remote")};
	/// false：解析链跳过 rules，便于单独验证本地/云端模型
	bool enableRules = true;
	AiRemoteLlmConfig remoteLlm;
	AiRouterConfig router;
	AiAgentPolicyConfig agent;
	std::vector<AiDomainModelConfig> domains;
};

#endif // CLOUDSIMAISDK_AICONFIGDTO_H
