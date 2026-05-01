#pragma once

#include "PropertyDescriptor.h"

#include <string>
#include <vector>

namespace property_core
{

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

inline void appendPack(PropertySchema& target, const std::vector<PropertyDescriptor>& packDescriptors)
{
	target.descriptors.insert(target.descriptors.end(), packDescriptors.begin(), packDescriptors.end());
}

} // namespace property_core
