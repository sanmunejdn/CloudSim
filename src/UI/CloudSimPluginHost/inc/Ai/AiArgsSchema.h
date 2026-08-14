#ifndef CLOUDSIMPLUGINHOST_AIARGSSCHEMA_H
#define CLOUDSIMPLUGINHOST_AIARGSSCHEMA_H

/// @file AiArgsSchema.h
/// @brief Catalog args_schema → JSON Schema / OpenAI tool parameters（单源）

#include <QByteArray>
#include <QString>
#include <QStringList>

#include <json.hpp>

namespace AiArgsSchema
{
/// args_schema 数组 → OpenAI function.parameters 对象
nlohmann::json toJsonSchemaParameters(const nlohmann::json& argsSchema);

/// 单条 Catalog api → OpenAI tools[] 元素；失败返回 null
nlohmann::json toOpenAiTool(const nlohmann::json& api);

QByteArray buildOpenAiToolsFromCatalog(const QByteArray& catalogJsonUtf8, const QString& domainId,
									   const QStringList& excludeApiIds = {});

/// 检查提案 args 是否缺 Catalog args_schema 中 required 字段；缺则写 outError 并返回 true
bool missingRequiredArgs(const nlohmann::json& argsSchema, const nlohmann::json& args, QString* outError);
} // namespace AiArgsSchema

#endif
