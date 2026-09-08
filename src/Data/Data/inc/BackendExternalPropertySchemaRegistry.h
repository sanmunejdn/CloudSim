#ifndef DATA_BACKENDEXTERNALPROPERTYSCHEMAREGISTRY_H
#define DATA_BACKENDEXTERNALPROPERTYSCHEMAREGISTRY_H

/// @file BackendExternalPropertySchemaRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 插件等外部类型向 Data schema 注册（避免 PluginHost↔Data 循环依赖）

#include "data_global.h"

#include "../../PropertyCore/inc/PropertySchema.h"

#include <string>

namespace backend_external_property_schema
{
DATA_EXPORT void registerSchema(const std::string& className, property_core::PropertySchema schema);
DATA_EXPORT void unregisterSchema(const std::string& className);
/// 清掉 schemaForClassName 对该 class 的缓存；空 className 清全部
DATA_EXPORT void invalidateSchemaCache(const std::string& className = {});
DATA_EXPORT bool tryGetSchema(const std::string& className, property_core::PropertySchema& out);
DATA_EXPORT const property_core::PropertySchema* findSchema(const std::string& className);
} // namespace backend_external_property_schema

namespace backend_property_binding
{
/// 由 External 注册表或 Binding 调用：剔除 className 缓存项
DATA_EXPORT void invalidateSchemaCacheForClass(const std::string& className);
DATA_EXPORT void invalidateAllSchemaCaches();
} // namespace backend_property_binding

#endif // DATA_BACKENDEXTERNALPROPERTYSCHEMAREGISTRY_H
