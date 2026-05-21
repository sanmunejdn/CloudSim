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

/// Blocking OpenAI-compatible chat completion; intended for JobSystem worker threads.
AIBACKEND_EXPORT LlmParseResult parseUserTextWithLlm(
	const QString& userText,
	const AiLlmConfig& config,
	const AiProgressSink& progress);
}
