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

/// 阻塞式 OpenAI 兼容对话；imagePng 非空时走 vision content；recognitionSchema 用于几何识别域
AIBACKEND_EXPORT LlmParseResult parseUserTextWithLlm(
	const QString& userText,
	const AiLlmConfig& config,
	const AiProgressSink& progress,
	const QByteArray& imagePng = QByteArray(),
	bool recognitionSchema = false);
}
