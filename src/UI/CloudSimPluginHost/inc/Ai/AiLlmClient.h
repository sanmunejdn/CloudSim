#pragma once

#include "aibackend_global.h"
#include "AiLlmConfig.h"
#include "AiProgressSink.h"

#include <json.hpp>
#include <QString>

namespace AiLlmClient
{
struct LlmParseResult
{
	bool ok = false;
	nlohmann::json command;
	QString errorMessage;
};

/// 阻塞式 OpenAI 兼容对话；domainId 选择 prompt/schema（mesh.create / mesh.compose / geometry.recognize）
AIBACKEND_EXPORT LlmParseResult parseUserTextWithLlm(
	const QString& userText,
	const AiLlmConfig& config,
	const AiProgressSink& progress,
	const QByteArray& imagePng = QByteArray(),
	const QString& domainId = QString());
}
