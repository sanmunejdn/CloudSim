#ifndef PROPERTYCORE_PROPERTYTYPES_H
#define PROPERTYCORE_PROPERTYTYPES_H

/// @file PropertyTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 属性提交后语义位：OSG 选中、follow 图、撤销钩子等

#include <array>
#include <cstdint>
#include <string>
#include <variant>

namespace property_core
{
/// 属性提交后语义位：OSG 选中、follow 图、撤销钩子等
enum class PropertySemanticFlags : std::uint32_t
{
	None = 0,
	/// 位姿/旋转影响后端世界变换与 OSG 选中
	AffectsBackendRootWorldXform = 1u << 0,
	/// 颜色影响 OSG 选中色
	AffectsColorOnly = 1u << 1,
	/// follow 目标/约束副作用
	AffectsFollowConstraintGraph = 1u << 2,
	/// 机器人指令运动字段（规划/轴刷新）
	AffectsInstructionMotion = 1u << 3,
	/// 非 schema 键：保留旧版全量提交（OSG + follow dirty）
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

#endif // PROPERTYCORE_PROPERTYTYPES_H
