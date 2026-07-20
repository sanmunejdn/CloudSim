#ifndef PROPERTYCORE_PROPERTYDESCRIPTOR_H
#define PROPERTYCORE_PROPERTYDESCRIPTOR_H

/// @file PropertyDescriptor.h
/// @brief 双精度滑条范围（enabled 才生效）

#include "PropertyTypes.h"

#include <limits>
#include <string>
#include <vector>

namespace property_core
{
/// 双精度滑条范围（enabled 才生效）
struct NumericRangeDouble
{
	bool enabled = false;
	double minValue = std::numeric_limits<double>::lowest();
	double maxValue = std::numeric_limits<double>::max();
};

struct NumericRangeInt
{
	bool enabled = false;
	int minValue = std::numeric_limits<int>::min();
	int maxValue = std::numeric_limits<int>::max();
};

/// 枚举下拉选项与是否允许自定义
struct EnumConstraint
{
	std::vector<std::string> options;
	bool allowCustom = true;
};

struct PropertyConstraints
{
	NumericRangeDouble rangeDouble;
	NumericRangeInt rangeInt;
	EnumConstraint enumConstraint;
};

/// 面板分组与编辑器类型提示
struct PropertyUiHint
{
	std::string editor;
	std::string group;
	int order = 0;
};

struct PropertyDescriptor
{
	std::string key;
	std::string label;
	PropertyType type = PropertyType::String;
	PropertyValue defaultValue{};
	bool editable = true;
	PropertyConstraints constraints;
	PropertyUiHint uiHint;
	PropertySemanticFlags semanticFlags = PropertySemanticFlags::None;
	/// 仅展示单位（如 mm、deg）；数值校验用 constraints
	std::string unit;
};

} // namespace property_core

#endif // PROPERTYCORE_PROPERTYDESCRIPTOR_H
