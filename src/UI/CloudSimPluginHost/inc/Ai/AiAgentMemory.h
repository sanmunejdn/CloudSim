#ifndef CLOUDSIMPLUGINHOST_AIAGENTMEMORY_H
#define CLOUDSIMPLUGINHOST_AIAGENTMEMORY_H

/// @file AiAgentMemory.h
/// @brief 会话步骤 + 按文档/api 记忆常用参数

#include <QByteArray>
#include <QString>
#include <QVariantMap>

namespace AiAgentMemory
{
QString memoryFilePath(const QString& applicationDirPath);

/// documentId 空则写全局 prefs；非空写 prefs_by_doc[documentId]
QVariantMap loadPrefs(const QString& applicationDirPath, const QString& documentId = QString());
void savePrefForApi(const QString& applicationDirPath, const QString& apiId, const QVariantMap& args,
					const QString& documentId = QString());

QByteArray sessionSummaryUtf8();
void appendSessionStep(const QString& toolId, bool ok, const QString& summary);
void clearSession();
} // namespace AiAgentMemory

#endif
