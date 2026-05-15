#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <variant>

namespace property_core
{

/// Bitmask: post-commit side effects (OSG selection, follow graph, undo hooks).
enum class PropertySemanticFlags : std::uint32_t
{
	None = 0,
	/// Pose or rotation scalars affect backend world pose / OSG selection (see syncOuterPatFromBackend).
	AffectsBackendRootWorldXform = 1u << 0,
	/// Color scalars affect OSG selected color display.
	AffectsColorOnly = 1u << 1,
	/// Follow target / constraint side effects (afterBackendFollowPropertyEdited).
	AffectsFollowConstraintGraph = 1u << 2,
	/// Robot instruction motion fields (planner / pose axes refresh).
	AffectsInstructionMotion = 1u << 3,
	/// Property key not in backend schema: preserve full legacy commit (OSG sync + follow dirty).
	LegacyFullCommitBehavior = 1u << 31
};

constexpr PropertySemanticFlags operator|(PropertySemanticFlags a, PropertySemanticFlags b)
{
	return static_cast<PropertySemanticFlags>(static_cast<std::uint32_t>(a) | static_cast<std::uint32_t>(b));
}

constexpr PropertySemanticFlags operator&(PropertySemanticFlags a, PropertySemanticFlags b)
{
	return static_cast<PropertySemanticFlags>(static_cast<std::uint32_t>(a) & static_cast<std::uint32_t>(b));
}

constexpr std::uint32_t semanticFlagsBits(PropertySemanticFlags f)
{
	return static_cast<std::uint32_t>(f);
}

enum class PropertyType
{
	Bool = 0,
	Int,
	Double,
	String,
	Enum,
	Vec3,
	ColorRgba
};

using Vec3Value = std::array<double, 3>;
using ColorRgbaValue = std::array<float, 4>;
using PropertyValue = std::variant<std::monostate, bool, int, double, std::string, Vec3Value, ColorRgbaValue>;

inline PropertyType propertyTypeOf(const PropertyValue& value)
{
	if (std::holds_alternative<bool>(value))
	{
		return PropertyType::Bool;
	}
	if (std::holds_alternative<int>(value))
	{
		return PropertyType::Int;
	}
	if (std::holds_alternative<double>(value))
	{
		return PropertyType::Double;
	}
	if (std::holds_alternative<std::string>(value))
	{
		return PropertyType::String;
	}
	if (std::holds_alternative<Vec3Value>(value))
	{
		return PropertyType::Vec3;
	}
	if (std::holds_alternative<ColorRgbaValue>(value))
	{
		return PropertyType::ColorRgba;
	}
	return PropertyType::String;
}

} // namespace property_core
