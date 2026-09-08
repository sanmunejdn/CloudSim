#ifndef CLOUDSIMPLUGINHOST_PLUGINPROPERTYBINDINGREGISTRY_H
#define CLOUDSIMPLUGINHOST_PLUGINPROPERTYBINDINGREGISTRY_H

/// @file PluginPropertyBindingRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 插件 Path B 属性 Binding 运行时表

#include "PluginBackendMeta.h"

#include <string>
#include <vector>

namespace plugin_property_binding_registry
{
struct Entry
{
	bool supportsTransform = true;
	bool supportsVisibility = true;
	std::vector<PluginPropertyBindingEntry> bindings;
};

void registerType(const std::string& className, Entry entry);
void unregisterType(const std::string& className);
bool tryGet(const std::string& className, Entry& out);
bool hasBindings(const std::string& className);
} // namespace plugin_property_binding_registry

#endif // CLOUDSIMPLUGINHOST_PLUGINPROPERTYBINDINGREGISTRY_H
