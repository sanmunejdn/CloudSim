#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include <cstddef>
#include <string>
#include <vector>

class IPluginSceneBridge;

/// Per-document tab API exposed to plugins.
class IPluginDocument
{
public:
	virtual ~IPluginDocument() = default;

	virtual std::string documentLabel() const = 0;
	virtual std::size_t backendObjectCount() const = 0;
	virtual std::vector<std::string> backendIds() const = 0;
	virtual bool containsBackend(const std::string& backendId) const = 0;
	virtual std::string backendDisplayName(const std::string& backendId) const = 0;
	virtual std::string backendClassName(const std::string& backendId) const = 0;

	virtual IPluginSceneBridge* sceneBridge() = 0;
	virtual const IPluginSceneBridge* sceneBridge() const = 0;
};
