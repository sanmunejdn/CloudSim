/// @file BackendExternalPropertySchemaRegistry.cpp
/// @brief 外部 schema 注册表

#include "BackendExternalPropertySchemaRegistry.h"
#include "BackendPropertyBinding.h"

#include <mutex>
#include <unordered_map>

namespace
{
std::mutex& registryMutex()
{
	static std::mutex mu;
	return mu;
}

std::unordered_map<std::string, property_core::PropertySchema>& schemaMap()
{
	static std::unordered_map<std::string, property_core::PropertySchema> m;
	return m;
}
} // namespace

namespace backend_external_property_schema
{
void registerSchema(const std::string& className, property_core::PropertySchema schema)
{
	if (className.empty())
	{
		return;
	}
	{
		std::lock_guard<std::mutex> lock(registryMutex());
		schemaMap()[className] = std::move(schema);
	}
	backend_property_binding::invalidateSchemaCacheForClass(className);
}

void unregisterSchema(const std::string& className)
{
	if (className.empty())
	{
		return;
	}
	{
		std::lock_guard<std::mutex> lock(registryMutex());
		schemaMap().erase(className);
	}
	backend_property_binding::invalidateSchemaCacheForClass(className);
}

void invalidateSchemaCache(const std::string& className)
{
	if (className.empty())
	{
		backend_property_binding::invalidateAllSchemaCaches();
	}
	else
	{
		backend_property_binding::invalidateSchemaCacheForClass(className);
	}
}

const property_core::PropertySchema* findSchema(const std::string& className)
{
	std::lock_guard<std::mutex> lock(registryMutex());
	const auto it = schemaMap().find(className);
	if (it == schemaMap().end())
	{
		return nullptr;
	}
	return &it->second;
}

bool tryGetSchema(const std::string& className, property_core::PropertySchema& out)
{
	std::lock_guard<std::mutex> lock(registryMutex());
	const auto it = schemaMap().find(className);
	if (it == schemaMap().end())
	{
		return false;
	}
	out = it->second;
	return true;
}
} // namespace backend_external_property_schema
