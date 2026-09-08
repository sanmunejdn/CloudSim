#ifndef CLOUDSIMPLUGINSDK_PLUGINBACKENDMETA_H
#define CLOUDSIMPLUGINSDK_PLUGINBACKENDMETA_H

/// @file PluginBackendMeta.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 插件后端工厂（宿主适配 BackendDataBase）+ Binding 登记（1.54.0+）

#include "cloudsim_plugin_sdk_global.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

class IPluginBackendObject;

using PluginBackendFactory = std::function<std::shared_ptr<IPluginBackendObject>()>;

using PluginPropertyRowsProvider = std::function<std::string(const std::string& backendId)>;

/// 插件属性 Binding 条目（不链 Data；Host 转 schema / apply）
struct PluginPropertyBindingEntry
{
	const char* key = nullptr;
	const char* label = nullptr;
	int propertyType = 0; // property_core::PropertyType 底层 int
	bool editable = true;
	unsigned semanticFlags = 0;
	std::string (*formatValue)(const IPluginBackendObject*) = nullptr;
	bool (*applyValue)(IPluginBackendObject*, const char* valueUtf8, std::string* err) = nullptr;
};

struct PluginBackendMeta
{
	std::string className;
	std::string displayName;
	PluginBackendFactory factory;
	PluginPropertyRowsProvider propertyRowsProvider;
	bool supportsTransform = true;
	bool supportsVisibility = true;
	/// 非空时走 Binding 真源；空则回退 propertyRowsJson
	std::vector<PluginPropertyBindingEntry> propertyBindings;
};

class IPluginBackendObject
{
public:
	virtual ~IPluginBackendObject() = default;

	virtual std::string id() const = 0;
	virtual std::string name() const = 0;
	virtual std::string className() const = 0;
	virtual std::string propertyRowsJson() const = 0;
	virtual bool applyPropertyChange(const std::string& key, const std::string& valueUtf8) = 0;
};

#endif // CLOUDSIMPLUGINSDK_PLUGINBACKENDMETA_H
