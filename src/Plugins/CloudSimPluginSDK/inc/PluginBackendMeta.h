#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include <functional>
#include <memory>
#include <string>

class IPluginBackendObject;

/// Factory for a plugin-defined backend object (host adapts to \c BackendDataBase).
using PluginBackendFactory = std::function<std::shared_ptr<IPluginBackendObject>()>;

/// Property rows JSON provider (same schema as host property panel plugins).
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

/// Minimal backend surface for plugin-defined types (Phase 2).
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
