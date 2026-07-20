#ifndef PROPERTYCORE_PROPERTYBAG_H
#define PROPERTYCORE_PROPERTYBAG_H

/// @file PropertyBag.h
/// @brief schema 驱动的属性值存储（key 字符串）

#include "PropertyAttributeHelpers.h"
#include "PropertySchema.h"

#include <unordered_map>

#include <json.hpp>

namespace property_core
{
/// schema 驱动的属性值存储（key 字符串）
class PropertyBag
{
public:
	bool has(const std::string& key) const { return m_values.find(key) != m_values.end(); }

	const PropertyValue* get(const std::string& key) const
	{
		auto it = m_values.find(key);
		if (it == m_values.end())
		{
			return nullptr;
		}
		return &it->second;
	}

	void set(const std::string& key, PropertyValue value) { m_values[key] = std::move(value); }

	void seedDefaults(const PropertySchema& schema)
	{
		for (const PropertyDescriptor& descriptor : schema.descriptors)
		{
			if (!has(descriptor.key))
			{
				set(descriptor.key, descriptor.defaultValue);
			}
		}
	}

	bool validateAndSet(const PropertySchema& schema, const std::string& key, const PropertyValue& value,
						std::string* errMsg)
	{
		const PropertyDescriptor* descriptor = schema.find(key);
		if (!descriptor)
		{
			if (errMsg)
			{
				*errMsg = "Unknown property key.";
			}
			return false;
		}
		if (!validateAgainstDescriptor(*descriptor, value, errMsg))
		{
			return false;
		}
		set(key, value);
		return true;
	}

	nlohmann::json toJson(const PropertySchema& schema) const
	{
		nlohmann::json out = nlohmann::json::object();
		for (const PropertyDescriptor& descriptor : schema.descriptors)
		{
			const PropertyValue* value = get(descriptor.key);
			if (!value)
			{
				continue;
			}
			out[descriptor.key] = valueToJson(descriptor.type, *value);
		}
		return out;
	}

	bool loadFromJson(const nlohmann::json& input, const PropertySchema& schema, std::string* errMsg)
	{
		for (const PropertyDescriptor& descriptor : schema.descriptors)
		{
			if (!input.contains(descriptor.key))
			{
				continue;
			}
			PropertyValue value;
			if (!jsonToValue(descriptor.type, input[descriptor.key], value, errMsg))
			{
				return false;
			}
			if (!validateAndSet(schema, descriptor.key, value, errMsg))
			{
				return false;
			}
		}
		return true;
	}

private:
	static bool validateAgainstDescriptor(const PropertyDescriptor& descriptor, const PropertyValue& value,
										  std::string* errMsg)
	{
		switch (descriptor.type)
		{
		case PropertyType::Bool:
			return std::holds_alternative<bool>(value);
		case PropertyType::Int:
			if (!std::holds_alternative<int>(value))
			{
				return false;
			}
			if (descriptor.constraints.rangeInt.enabled)
			{
				const int v = std::get<int>(value);
				return v >= descriptor.constraints.rangeInt.minValue && v <= descriptor.constraints.rangeInt.maxValue;
			}
			return true;
		case PropertyType::Double:
			if (!std::holds_alternative<double>(value))
			{
				return false;
			}
			if (descriptor.constraints.rangeDouble.enabled)
			{
				const double v = std::get<double>(value);
				return v >= descriptor.constraints.rangeDouble.minValue &&
					   v <= descriptor.constraints.rangeDouble.maxValue;
			}
			return true;
		case PropertyType::String:
			return std::holds_alternative<std::string>(value);
		case PropertyType::Enum:
			if (!std::holds_alternative<std::string>(value))
			{
				return false;
			}
			if (descriptor.constraints.enumConstraint.allowCustom)
			{
				return true;
			}
			for (const std::string& option : descriptor.constraints.enumConstraint.options)
			{
				if (option == std::get<std::string>(value))
				{
					return true;
				}
			}
			return false;
		case PropertyType::Vec3:
			return std::holds_alternative<Vec3Value>(value);
		case PropertyType::ColorRgba:
			return std::holds_alternative<ColorRgbaValue>(value);
		default:
			if (errMsg)
			{
				*errMsg = "Unsupported property type.";
			}
			return false;
		}
	}

	static nlohmann::json valueToJson(PropertyType type, const PropertyValue& value)
	{
		switch (type)
		{
		case PropertyType::Bool:
			return std::get<bool>(value);
		case PropertyType::Int:
			return std::get<int>(value);
		case PropertyType::Double:
			return std::get<double>(value);
		case PropertyType::String:
		case PropertyType::Enum:
			return std::get<std::string>(value);
		case PropertyType::Vec3:
		{
			const Vec3Value vec = std::get<Vec3Value>(value);
			return nlohmann::json::array({vec[0], vec[1], vec[2]});
		}
		case PropertyType::ColorRgba:
		{
			const ColorRgbaValue rgba = std::get<ColorRgbaValue>(value);
			return nlohmann::json::array({rgba[0], rgba[1], rgba[2], rgba[3]});
		}
		default:
			return nlohmann::json();
		}
	}

	static bool jsonToValue(PropertyType type, const nlohmann::json& jsonValue, PropertyValue& out, std::string* errMsg)
	{
		try
		{
			switch (type)
			{
			case PropertyType::Bool:
				out = jsonValue.get<bool>();
				return true;
			case PropertyType::Int:
				out = jsonValue.get<int>();
				return true;
			case PropertyType::Double:
				out = jsonValue.get<double>();
				return true;
			case PropertyType::String:
			case PropertyType::Enum:
				out = jsonValue.get<std::string>();
				return true;
			case PropertyType::Vec3:
				if (!jsonValue.is_array() || jsonValue.size() != 3)
				{
					return false;
				}
				out = Vec3Value{jsonValue[0].get<double>(), jsonValue[1].get<double>(), jsonValue[2].get<double>()};
				return true;
			case PropertyType::ColorRgba:
				if (!jsonValue.is_array() || jsonValue.size() != 4)
				{
					return false;
				}
				out = ColorRgbaValue{jsonValue[0].get<float>(), jsonValue[1].get<float>(), jsonValue[2].get<float>(),
									 jsonValue[3].get<float>()};
				return true;
			default:
				return false;
			}
		}
		catch (...)
		{
			if (errMsg)
			{
				*errMsg = "Invalid property json value.";
			}
			return false;
		}
	}

	std::unordered_map<std::string, PropertyValue> m_values;
};

} // namespace property_core

#endif // PROPERTYCORE_PROPERTYBAG_H
