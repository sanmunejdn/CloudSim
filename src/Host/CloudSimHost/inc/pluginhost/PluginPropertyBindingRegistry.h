#ifndef CLOUDSIMHOST_PLUGINPROPERTYBINDINGREGISTRY_H
#define CLOUDSIMHOST_PLUGINPROPERTYBINDINGREGISTRY_H

/// @file PluginPropertyBindingRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 插件 Path B 属性 Binding 运行时表

#include "cloudsim_host_global.h"
#include "PluginBackendMeta.h"

#include <string>
#include <vector>

// Viewport 自测链入 Core.obj 本地定义；Host/Headless 经 WHOLEARCHIVE 导出
#if defined(CLOUDSIM_VIEWPORT_LIB)
#define PPBR_API
#else
#define PPBR_API CLOUDSIM_HOST_EXPORT
#endif

namespace plugin_property_binding_registry
{
struct Entry
{
	bool supportsTransform = true;
	bool supportsVisibility = true;
	std::vector<PluginPropertyBindingEntry> bindings;
};

PPBR_API void registerType(const std::string& className, Entry entry);
PPBR_API void unregisterType(const std::string& className);
PPBR_API bool tryGet(const std::string& className, Entry& out);
PPBR_API bool hasBindings(const std::string& className);
} // namespace plugin_property_binding_registry

#endif // CLOUDSIMHOST_PLUGINPROPERTYBINDINGREGISTRY_H
