/// @file PluginPropertyBindingRegistry.cpp
/// @brief 插件 Binding 运行时表

#include "PluginPropertyBindingRegistry.h"

#include "BackendExternalPropertySchemaRegistry.h"

#include "../../Data/PropertyCore/inc/PropertyTypes.h"

#include <mutex>
#include <unordered_map>

namespace
{
std::mutex& mu()
{
	static std::mutex m;
	return m;
}
std::unordered_map<std::string, plugin_property_binding_registry::Entry>& map()
{
	static std::unordered_map<std::string, plugin_property_binding_registry::Entry> m;
	return m;
}

property_core::PropertySchema schemaFromEntries(const std::string& className,
												const std::vector<PluginPropertyBindingEntry>& bindings)
{
	property_core::PropertySchema s;
	s.objectTypeId = "plugin." + className;
	s.schemaVersion = 1;
	for (const PluginPropertyBindingEntry& e : bindings)
	{
		if (!e.key)
		{
			continue;
		}
		property_core::PropertyDescriptor d;
		d.key = e.key;
		d.label = e.label ? e.label : e.key;
		d.type = static_cast<property_core::PropertyType>(e.propertyType);
		d.editable = e.editable;
		d.semanticFlags = static_cast<property_core::PropertySemanticFlags>(e.semanticFlags);
		s.descriptors.push_back(std::move(d));
	}
	return s;
}
} // namespace

namespace plugin_property_binding_registry
{
void registerType(const std::string& className, Entry entry)
{
	if (className.empty())
	{
		return;
	}
	property_core::PropertySchema schema = schemaFromEntries(className, entry.bindings);
	{
		std::lock_guard<std::mutex> lock(mu());
		map()[className] = std::move(entry);
	}
	backend_external_property_schema::registerSchema(className, std::move(schema));
}

void unregisterType(const std::string& className)
{
	{
		std::lock_guard<std::mutex> lock(mu());
		map().erase(className);
	}
	backend_external_property_schema::unregisterSchema(className);
}

const Entry* find(const std::string& className)
{
	std::lock_guard<std::mutex> lock(mu());
	const auto it = map().find(className);
	if (it == map().end())
	{
		return nullptr;
	}
	return &it->second;
}

bool tryGet(const std::string& className, Entry& out)
{
	std::lock_guard<std::mutex> lock(mu());
	const auto it = map().find(className);
	if (it == map().end())
	{
		return false;
	}
	out = it->second;
	return true;
}

bool hasBindings(const std::string& className)
{
	std::lock_guard<std::mutex> lock(mu());
	const auto it = map().find(className);
	return it != map().end() && !it->second.bindings.empty();
}
} // namespace plugin_property_binding_registry
