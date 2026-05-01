#pragma once

#include <array>
#include <string>
#include <variant>

namespace property_core
{

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
