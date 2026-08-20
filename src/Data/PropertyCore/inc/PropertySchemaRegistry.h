#ifndef PROPERTYCORE_PROPERTYSCHEMAREGISTRY_H
#define PROPERTYCORE_PROPERTYSCHEMAREGISTRY_H

/// @file PropertySchemaRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief objectTypeId → PropertySchema 全局表

#include "PropertySchema.h"

#include <unordered_map>

namespace property_core
{
/// objectTypeId → PropertySchema 全局表
class PropertySchemaRegistry
{
public:
	bool registerSchema(PropertySchema schema)
	{
		if (schema.objectTypeId.empty())
		{
			return false;
		}
		m_schemas[schema.objectTypeId] = std::move(schema);
		return true;
	}

	const PropertySchema* find(const std::string& objectTypeId) const
	{
		auto it = m_schemas.find(objectTypeId);
		if (it == m_schemas.end())
		{
			return nullptr;
		}
		return &it->second;
	}

private:
	std::unordered_map<std::string, PropertySchema> m_schemas;
};

} // namespace property_core

#endif // PROPERTYCORE_PROPERTYSCHEMAREGISTRY_H
