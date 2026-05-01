#pragma once

#include "PropertyTypes.h"

#include <limits>
#include <string>
#include <vector>

namespace property_core
{

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
};

} // namespace property_core
