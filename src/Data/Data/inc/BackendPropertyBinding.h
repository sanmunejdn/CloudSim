#ifndef DATA_BACKENDPROPERTYBINDING_H
#define DATA_BACKENDPROPERTYBINDING_H

/// @file BackendPropertyBinding.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 面板/schema 唯一属性清单：descriptor + 字符串 format/apply

#include "data_global.h"

#include "../../PropertyCore/inc/PropertyDescriptor.h"
#include "../../PropertyCore/inc/PropertySchema.h"

#include <string>
#include <vector>

#include <json.hpp>

class BackendDataBase;

struct DATA_EXPORT BackendPropertyBinding
{
	property_core::PropertyDescriptor desc;
	std::string (*formatValue)(const BackendDataBase&) = nullptr;
	bool (*applyValue)(BackendDataBase&, const std::string& value, std::string* err) = nullptr;
};

namespace backend_property_binding
{
/// 按 has* 能力 + extraPropertyBindings 组装完整清单
DATA_EXPORT std::vector<BackendPropertyBinding> collectBindings(const BackendDataBase& data);

/// 由 Binding 表生成 schema（按 className 缓存）
DATA_EXPORT const property_core::PropertySchema& schemaForClassName(const std::string& className);

DATA_EXPORT void appendBindingRows(const BackendDataBase& data, nlohmann::json& rows);

DATA_EXPORT bool applyBindingKey(BackendDataBase& data, const std::string& key, const std::string& value,
								 std::string* errMsg);

} // namespace backend_property_binding

namespace backend_property_binding_extras
{
DATA_EXPORT const std::vector<BackendPropertyBinding>& meshExtras();
DATA_EXPORT const std::vector<BackendPropertyBinding>& axisLengthExtras();
DATA_EXPORT const std::vector<BackendPropertyBinding>& emptyExtras();
} // namespace backend_property_binding_extras

#endif // DATA_BACKENDPROPERTYBINDING_H
