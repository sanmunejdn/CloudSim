#ifndef CLOUDSIMPLUGINHOST_AICOMMANDSCHEMA_H
#define CLOUDSIMPLUGINHOST_AICOMMANDSCHEMA_H

/// @file AiCommandSchema.h
/// @brief 校验 create_mesh 并填充 mesh 参数

#include "aibackend_global.h"

#include "BackendPrimitiveGeometry.h"

#include <optional>
#include <string>

#include <json.hpp>

namespace AiCommandSchema
{
constexpr int kSchemaVersion = 1;
constexpr double kMinDimMm = 0.1;
constexpr double kMaxDimMm = 1e6;

AIBACKEND_EXPORT bool primitiveKindFromString(const std::string& s, BackendPrimitiveGeometry::PrimitiveKind& out);
AIBACKEND_EXPORT std::string primitiveKindToString(BackendPrimitiveGeometry::PrimitiveKind kind);

/// 校验 create_mesh 并填充 mesh 参数
AIBACKEND_EXPORT bool parseCreateMeshCommand(const nlohmann::json& cmd,
											 BackendPrimitiveGeometry::PrimitiveMeshParams& outParams,
											 BackendPrimitiveGeometry::PrimitiveMeshQuality& outQuality,
											 std::string& outDisplayName, std::string& outSourcePath,
											 std::string& errorMessage);

AIBACKEND_EXPORT std::string defaultDisplayNameFor(const BackendPrimitiveGeometry::PrimitiveMeshParams& params);

/// 从 LLM 原文提取 JSON 对象文本（去 markdown 围栏）
AIBACKEND_EXPORT std::string extractJsonObjectText(const std::string& text);

/// 修复 mesh.compose 常见 LLM 语法：steps 数组内误写为 "stepId":{...}
AIBACKEND_EXPORT std::string repairComposePlanJsonText(const std::string& text);

/// 规范化已解析的 compose ActionPlan（id、dimensions_mm 等）
AIBACKEND_EXPORT void normalizeComposePlanJson(nlohmann::json& root);

/// 从 LLM 原文提取 create_mesh JSON（可含 markdown 围栏）
AIBACKEND_EXPORT bool tryParseCreateMeshCommandJson(const std::string& llmText, nlohmann::json& outCommand,
													std::string& errorMessage);

/// 校验 modify_object（propertyBag / pose 补丁，执行由 Host 计划器承接）
AIBACKEND_EXPORT bool parseModifyObjectCommand(const nlohmann::json& cmd, std::string& outBackendId,
											   nlohmann::json& outPropertyPatch, std::string& errorMessage);

/// 校验 import_asset（文件路径）
AIBACKEND_EXPORT bool parseImportAssetCommand(const nlohmann::json& cmd, std::string& outFilePath,
											  std::string& errorMessage);

} // namespace AiCommandSchema

#endif // CLOUDSIMPLUGINHOST_AICOMMANDSCHEMA_H
