#ifndef PROPERTYCORE_PROPERTYATTRIBUTEHELPERS_H
#define PROPERTYCORE_PROPERTYATTRIBUTEHELPERS_H

/// @file PropertyAttributeHelpers.h
/// @brief 属性 JSON 行格式化与解析工具

#include <array>
#include <cctype>
#include <limits>
#include <sstream>
#include <string>
#include <type_traits>

#include <json.hpp>

/// 属性 JSON 行格式化与解析工具
namespace property_core
{
inline std::string formatDoubleFixed3(double value)
{
	std::ostringstream oss;
	oss.setf(std::ios::fixed);
	oss.precision(3);
	oss << value;
	return oss.str();
}

inline bool parseStrictDouble(const std::string& text, double& out, std::string* errMsg)
{
	try
	{
		std::size_t index = 0;
		out = std::stod(text, &index);
		while (index < text.size() && (text[index] == ' ' || text[index] == '\t'))
		{
			++index;
		}
		if (index != text.size())
		{
			if (errMsg)
			{
				*errMsg = "Invalid number.";
			}
			return false;
		}
		return true;
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "Invalid number.";
		}
		return false;
	}
}

inline bool parseStrictInt(const std::string& text, int& out, std::string* errMsg)
{
	try
	{
		std::size_t index = 0;
		const long long parsed = std::stoll(text, &index, 10);
		while (index < text.size() && (text[index] == ' ' || text[index] == '\t'))
		{
			++index;
		}
		if (index != text.size() || parsed < static_cast<long long>(std::numeric_limits<int>::min()) ||
			parsed > static_cast<long long>(std::numeric_limits<int>::max()))
		{
			if (errMsg)
			{
				*errMsg = "Invalid integer.";
			}
			return false;
		}
		out = static_cast<int>(parsed);
		return true;
	}
	catch (...)
	{
		if (errMsg)
		{
			*errMsg = "Invalid integer.";
		}
		return false;
	}
}

inline bool parseStrictBool(const std::string& text, bool& out, std::string* errMsg)
{
	std::string normalized;
	normalized.reserve(text.size());
	for (unsigned char ch : text)
	{
		if (ch == ' ' || ch == '\t')
		{
			continue;
		}
		normalized.push_back(static_cast<char>(std::tolower(ch)));
	}
	if (normalized == "1" || normalized == "true" || normalized == "yes" || normalized == "on")
	{
		out = true;
		return true;
	}
	if (normalized == "0" || normalized == "false" || normalized == "no" || normalized == "off")
	{
		out = false;
		return true;
	}
	if (errMsg)
	{
		*errMsg = "Invalid boolean.";
	}
	return false;
}

template <typename TValue>
inline std::string formatScalarValue(const TValue& value)
{
	if constexpr (std::is_same_v<TValue, double>)
	{
		return formatDoubleFixed3(value);
	}
	else if constexpr (std::is_same_v<TValue, int>)
	{
		return std::to_string(value);
	}
	else if constexpr (std::is_same_v<TValue, bool>)
	{
		return value ? "true" : "false";
	}
	else if constexpr (std::is_same_v<TValue, std::string>)
	{
		return value;
	}
	else
	{
		static_assert(!sizeof(TValue), "Unsupported scalar value type.");
	}
}

template <typename TValue>
inline bool parseScalarValue(const std::string& text, TValue& out, std::string* errMsg)
{
	if constexpr (std::is_same_v<TValue, double>)
	{
		return parseStrictDouble(text, out, errMsg);
	}
	else if constexpr (std::is_same_v<TValue, int>)
	{
		return parseStrictInt(text, out, errMsg);
	}
	else if constexpr (std::is_same_v<TValue, bool>)
	{
		return parseStrictBool(text, out, errMsg);
	}
	else if constexpr (std::is_same_v<TValue, std::string>)
	{
		out = text;
		return true;
	}
	else
	{
		static_assert(!sizeof(TValue), "Unsupported scalar value type.");
	}
}

template <std::size_t N>
inline bool containsKey(const std::array<const char*, N>& keys, const std::string& key)
{
	for (const char* candidate : keys)
	{
		if (key == candidate)
		{
			return true;
		}
	}
	return false;
}

template <typename TAppendRow, typename TVec3>
inline void appendVec3Rows(nlohmann::json& rows, const TVec3& vec, const std::array<const char*, 3>& keys,
						   const std::array<const char*, 3>& labels, TAppendRow appendRow)
{
	appendRow(rows, keys[0], labels[0], true, formatDoubleFixed3(static_cast<double>(vec.x)));
	appendRow(rows, keys[1], labels[1], true, formatDoubleFixed3(static_cast<double>(vec.y)));
	appendRow(rows, keys[2], labels[2], true, formatDoubleFixed3(static_cast<double>(vec.z)));
}

template <typename TVec3>
inline bool applyVec3ByKey(TVec3& vec, const std::array<const char*, 3>& keys, const std::string& key, double value)
{
	if (key == keys[0])
	{
		vec.x = value;
		return true;
	}
	if (key == keys[1])
	{
		vec.y = value;
		return true;
	}
	if (key == keys[2])
	{
		vec.z = value;
		return true;
	}
	return false;
}

template <typename TAppendRow, typename TColor>
inline void appendRgbaRows(nlohmann::json& rows, const TColor& color, const std::array<const char*, 4>& keys,
						   const std::array<const char*, 4>& labels, TAppendRow appendRow)
{
	appendRow(rows, keys[0], labels[0], true, formatDoubleFixed3(static_cast<double>(color.r)));
	appendRow(rows, keys[1], labels[1], true, formatDoubleFixed3(static_cast<double>(color.g)));
	appendRow(rows, keys[2], labels[2], true, formatDoubleFixed3(static_cast<double>(color.b)));
	appendRow(rows, keys[3], labels[3], true, formatDoubleFixed3(static_cast<double>(color.a)));
}

template <typename TColor>
inline bool applyRgbaByKey(TColor& color, const std::array<const char*, 4>& keys, const std::string& key, double value)
{
	if (key == keys[0])
	{
		color.r = static_cast<float>(value);
		return true;
	}
	if (key == keys[1])
	{
		color.g = static_cast<float>(value);
		return true;
	}
	if (key == keys[2])
	{
		color.b = static_cast<float>(value);
		return true;
	}
	if (key == keys[3])
	{
		color.a = static_cast<float>(value);
		return true;
	}
	return false;
}

} // namespace property_core

#endif // PROPERTYCORE_PROPERTYATTRIBUTEHELPERS_H
