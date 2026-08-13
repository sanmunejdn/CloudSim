#ifndef CLOUDSIMPLUGINHOST_AIINTENTCLASSIFIER_H
#define CLOUDSIMPLUGINHOST_AIINTENTCLASSIFIER_H

/// @file AiIntentClassifier.h
/// @brief 轻量意图分类：规则打分；可选 local LLM 选域

#include "AiConfigDto.h"

#include <QString>

namespace AiIntentClassifier
{
struct Result
{
	QString domainId;
	int score = 0;
	QString via;
};

/// 规则打分选域；score < minScore 时 domainId 为空
Result classifyByRules(const QString& userText, int minScore = 2);

/// Ollama/OpenAI 兼容：仅输出 domain id；失败返回空
Result classifyByLocalLlm(const QString& userText, const AiConfigDto& config);
} // namespace AiIntentClassifier

#endif
