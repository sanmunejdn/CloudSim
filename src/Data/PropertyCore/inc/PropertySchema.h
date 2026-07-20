#ifndef PROPERTYCORE_PROPERTYSCHEMA_H
#define PROPERTYCORE_PROPERTYSCHEMA_H

/// @file PropertySchema.h
/// @brief 某 objectTypeId 的属性描述表

#include "PropertyDescriptor.h"

#include <string>
#include <vector>

namespace property_core
{
/// 某 objectTypeId 的属性描述表
struct PropertySchema
{
	std::string objectTypeId;
	int schemaVersion = 1;
	std::vector<PropertyDescriptor> descriptors;

	const PropertyDescriptor* find(const std::string& key) const
	{
		for (const PropertyDescriptor& descriptor : descriptors)
		{
			if (descriptor.key == key)
			{
				return &descriptor;
			}
		}
		return nullptr;
	}
};

/// 合并 descriptor 包到 schema
inline void appendPack(PropertySchema& target, const std::vector<PropertyDescriptor>& packDescriptors)
{
	target.descriptors.insert(target.descriptors.end(), packDescriptors.begin(), packDescriptors.end());
}

} // namespace property_core

#endif // PROPERTYCORE_PROPERTYSCHEMA_H
