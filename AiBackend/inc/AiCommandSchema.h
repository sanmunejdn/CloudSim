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

/// Validates create_mesh command; fills mesh params and quality on success.
AIBACKEND_EXPORT bool parseCreateMeshCommand(
	const nlohmann::json& cmd,
	BackendPrimitiveGeometry::PrimitiveMeshParams& outParams,
	BackendPrimitiveGeometry::PrimitiveMeshQuality& outQuality,
	std::string& outDisplayName,
	std::string& outSourcePath,
	std::string& errorMessage);

AIBACKEND_EXPORT std::string defaultDisplayNameFor(const BackendPrimitiveGeometry::PrimitiveMeshParams& params);

/// Extract and validate create_mesh JSON from raw LLM text (may include markdown fences).
AIBACKEND_EXPORT bool tryParseCreateMeshCommandJson(
	const std::string& llmText,
	nlohmann::json& outCommand,
	std::string& errorMessage);

}