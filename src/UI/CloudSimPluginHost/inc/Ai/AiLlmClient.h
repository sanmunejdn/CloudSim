#ifndef CLOUDSIMPLUGINHOST_AILLMCLIENT_H
#define CLOUDSIMPLUGINHOST_AILLMCLIENT_H

/// @file AiLlmClient.h
/// @brief 阻塞式 OpenAI 兼容对话；domainId 选择 prompt/schema（mesh.create / mesh.compose / geometry.recognize）

#include "aibackend_global.h"

#include "AiLlmConfig.h"
#include "AiProgressSink.h"

#include <QString>

#include <json.hpp>

namespace AiLlmClient
{
struct LlmParseResult
{
	bool ok = false;
	nlohmann::json command;
	QString errorMessage;
};

/// 阻塞式 OpenAI 兼容对话；domainId 选择 prompt/schema（mesh.create / mesh.compose / geometry.recognize）
AIBACKEND_EXPORT LlmParseResult parseUserTextWithLlm(const QString& userText, const AiLlmConfig& config,
													 const AiProgressSink& progress,
													 const QByteArray& imagePng = QByteArray(),
													 const QString& domainId = QString(),
													 const QByteArray& catalogSliceUtf8 = QByteArray());
} // namespace AiLlmClient

#endif // CLOUDSIMPLUGINHOST_AILLMCLIENT_H
