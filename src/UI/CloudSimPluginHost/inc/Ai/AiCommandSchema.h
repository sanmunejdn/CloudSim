#pragma once

#include "aibackend_global.h"

#include <json.hpp>
#include <optional>
#include <string>

#include "BackendPrimitiveGeometry.h"

namespace AiCommandSchema
{
constexpr int kSchemaVersion = 1;
constexpr double kMinDimMm = 0.1;
constexpr double kMaxDimMm = 1e6;

AIBACKEND_EXPORT bool primitiveKindFromString(const std::string& s, BackendPrimitiveGeometry::PrimitiveKind& out);
AIBACKEND_EXPORT std::string primitiveKindToString(BackendPrimitiveGeometry::PrimitiveKind kind);

/// 校验 create_mesh 并填充 mesh 参数
AIBACKEND_EXPORT bool parseCreateMeshCommand(
	const nlohmann::json& cmd,
	BackendPrimitiveGeometry::PrimitiveMeshParams& outParams,
	BackendPrimitiveGeometry::PrimitiveMeshQuality& outQuality,
	std::string& outDisplayName,
	std::string& outSourcePath,
	std::string& errorMessage);

AIBACKEND_EXPORT std::string defaultDisplayNameFor(const BackendPrimitiveGeometry::PrimitiveMeshParams& params);

/// 从 LLM 原文提取 create_mesh JSON（可含 markdown 围栏）
AIBACKEND_EXPORT bool tryParseCreateMeshCommandJson(
	const std::string& llmText,
	nlohmann::json& outCommand,
	std::string& errorMessage);

}