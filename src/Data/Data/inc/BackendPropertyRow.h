#ifndef DATA_BACKENDPROPERTYROW_H
#define DATA_BACKENDPROPERTYROW_H

/// @file BackendPropertyRow.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 属性面板 JSON 行字段约定

#include <string>
#include <utility>

#include <json.hpp>

/// 属性面板 JSON 行字段约定
namespace backend_property_json
{
inline constexpr const char* kKey = "key";
inline constexpr const char* kLabelEn = "labelEn";
inline constexpr const char* kEditable = "editable";
inline constexpr const char* kValue = "value";

inline nlohmann::json makeRow(std::string key, std::string labelEn, bool editable, std::string value)
{
	nlohmann::json row;
	row[kKey] = std::move(key);
	row[kLabelEn] = std::move(labelEn);
	row[kEditable] = editable;
	row[kValue] = std::move(value);
	return row;
}

inline void appendRow(nlohmann::json& rowArray, std::string key, std::string labelEn, bool editable, std::string value)
{
	rowArray.push_back(makeRow(std::move(key), std::move(labelEn), editable, std::move(value)));
}

} // namespace backend_property_json

#endif // DATA_BACKENDPROPERTYROW_H
