#ifndef CLOUDSIMPLUGINHOST_AILLMCLIENT_H
#define CLOUDSIMPLUGINHOST_AILLMCLIENT_H

/// @file AiLlmClient.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 阻塞式 OpenAI 兼容对话；含 tools/tool_calls 多轮

#include "aibackend_global.h"

#include "AiAgentTypes.h"
#include "AiLlmConfig.h"
#include "AiProgressSink.h"

#include <QByteArray>
#include <QString>
#include <QStringList>

#include <json.hpp>

namespace AiLlmClient
{
struct LlmParseResult
{
	bool ok = false;
	nlohmann::json command;
	QString errorMessage;
};

struct ToolProposeResult
{
	bool ok = false;
	bool hasToolCall = false;
	QString toolName;
	QString toolCallId;
	nlohmann::json args = nlohmann::json::object();
	QString assistantText;
	QString errorMessage;
	nlohmann::json assistantMessage; // 写入多轮 messages
};

AIBACKEND_EXPORT LlmParseResult parseUserTextWithLlm(const QString& userText, const AiLlmConfig& config,
													 const AiProgressSink& progress,
													 const QByteArray& imagePng = QByteArray(),
													 const QString& domainId = QString(),
													 const QByteArray& catalogSliceUtf8 = QByteArray());

AIBACKEND_EXPORT QByteArray buildOpenAiToolsFromCatalog(const QByteArray& catalogJsonUtf8, const QString& domainId,
														const QStringList& excludeApiIds = {});

/// inoutMessages：多轮 messages[]；空则用 userText+snapshot 新建；成功后追加 assistant
AIBACKEND_EXPORT ToolProposeResult chatWithTools(const QString& userText, const AiLlmConfig& config,
												 const AiProgressSink& progress, const QByteArray& toolsJsonUtf8,
												 const QByteArray& sceneSnapshotUtf8 = QByteArray(),
												 const QByteArray& sessionSummaryUtf8 = QByteArray(),
												 nlohmann::json* inoutMessages = nullptr);

/// 将 tool 观测追加到 messages（OpenAI tool role）
AIBACKEND_EXPORT void appendToolObservation(nlohmann::json& messages, const QString& toolCallId, const QString& toolName,
											const QByteArray& observationUtf8);

struct PlanJsonResult
{
	bool ok = false;
	AiAgentPlan plan;
	QString errorMessage;
	QString rawText;
};

/// 将用户需求拆成有序 Catalog 步骤（非 tool_calls）
AIBACKEND_EXPORT PlanJsonResult chatPlanJson(const QString& userText, const AiLlmConfig& config,
											 const AiProgressSink& progress, const QByteArray& catalogJsonUtf8,
											 const QString& domainId, const QByteArray& sceneSnapshotUtf8 = QByteArray(),
											 const QByteArray& sessionSummaryUtf8 = QByteArray());

struct TextChatResult
{
	bool ok = false;
	QString assistantText;
	QString errorMessage;
};

/// 纯文本多轮（意图分类等）；无 tools
AIBACKEND_EXPORT TextChatResult chatText(const QString& systemPrompt, const QString& userText, const AiLlmConfig& config,
										 const AiProgressSink& progress = {});
} // namespace AiLlmClient

#endif
