#ifndef CLOUDSIMPLUGINSDK_PLUGINBACKENDMETA_H
#define CLOUDSIMPLUGINSDK_PLUGINBACKENDMETA_H

/// @file PluginBackendMeta.h
/// @brief 插件后端工厂（宿主适配 BackendDataBase）

#include "cloudsim_plugin_sdk_global.h"

#include <functional>
#include <memory>
#include <string>

class IPluginBackendObject;

/// 插件后端工厂（宿主适配 BackendDataBase）
using PluginBackendFactory = std::function<std::shared_ptr<IPluginBackendObject>()>;

/// 属性行 JSON 提供者（同宿主面板 schema）
using PluginPropertyRowsProvider = std::function<std::string(const std::string& backendId)>;

struct PluginBackendMeta
{
	std::string className;
	std::string displayName;
	PluginBackendFactory factory;
	PluginPropertyRowsProvider propertyRowsProvider;
	bool supportsTransform = true;
	bool supportsVisibility = true;
};

/// 插件自定义后端最小面（Phase 2）
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
