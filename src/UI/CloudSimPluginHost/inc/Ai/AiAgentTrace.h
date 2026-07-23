#ifndef CLOUDSIMPLUGINHOST_AIAGENTTRACE_H
#define CLOUDSIMPLUGINHOST_AIAGENTTRACE_H

/// @file AiAgentTrace.h
/// @brief Agent 步骤轨迹（exe 旁 ai_agent_trace.jsonl）

#include <QByteArray>
#include <QString>

namespace AiAgentTrace
{
QString traceFilePath(const QString& applicationDirPath);

void append(const QString& applicationDirPath, const QString& state, const QString& toolId, bool ok,
			const QByteArray& detailUtf8);
} // namespace AiAgentTrace

#endif
