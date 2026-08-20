#ifndef CLOUDSIMPLUGINHOST_AIAGENTPLANBUILDER_H
#define CLOUDSIMPLUGINHOST_AIAGENTPLANBUILDER_H

/// @file AiAgentPlanBuilder.h
/// @brief 规则优先、LLM JSON 兜底的需求拆分

#include "AiAgentTypes.h"
#include "AiConfigDto.h"
#include "AiLlmConfig.h"
#include "AiProgressSink.h"

#include <QByteArray>
#include <QString>
#include <QStringList>

namespace AiAgentPlanBuilder
{
struct BuildInput
{
	QString userText;
	QString domainId;
	QByteArray catalogJsonUtf8;
	QByteArray sceneSnapshotUtf8;
	QByteArray sessionSummaryUtf8;
	/// 已成功执行的 API，重规划时不得再纳入（避免「生成长方体」建成两次）
	QStringList excludeApiIds;
	int maxSteps = 8;
	bool enableLlmPlan = true;
	/// false：跳过场景/工艺/多 keyword 规则，只走 LLM 规划
	bool enableRules = true;
	AiLlmConfig llm;
	AiProgressSink progress;
};

/// 失败 observation 非空时表示对剩余目标重规划
AiAgentPlan buildPlan(const BuildInput& in, const QString& failureObservation = QString());
} // namespace AiAgentPlanBuilder

#endif
