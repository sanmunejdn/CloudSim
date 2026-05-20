#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include <QtPlugin>
#include <QString>

class IPluginHostContext;

/// Contract implemented by every CloudSim dynamic plugin DLL.
class ICloudSimPlugin
{
public:
	virtual ~ICloudSimPlugin() = default;

	virtual QString pluginId() const = 0;
	virtual QString displayName() const = 0;

	/// Called on the UI thread after manifest validation. Return false to skip the plugin.
	virtual bool initialize(IPluginHostContext* host) = 0;

	/// Called on the UI thread before application exit (plugins are not unloaded at runtime).
	virtual void shutdown() = 0;
};

#define CloudSimPlugin_iid "com.cloudsim.ICloudSimPlugin/1.0"
Q_DECLARE_INTERFACE(ICloudSimPlugin, CloudSimPlugin_iid)
