#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include "PluginPointCloudTypes.h"

#include <cstddef>
#include <string>
#include <vector>

class IPluginSceneBridge;

/// 插件可见的单文档页 API
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

	/// 稳定文档 id（同 EventHub documentId）
	virtual std::string documentId() const = 0;

	/// 经 IDataService 删子树（OSG + 事件）
	virtual bool removeBackendObject(const std::string& backendIdUtf8, std::string* outError = nullptr) = 0;

	virtual IPluginSceneBridge* sceneBridge() = 0;
	virtual const IPluginSceneBridge* sceneBridge() const = 0;

	/// 1.2.0+：点云元信息（UI 线程）
	virtual bool queryPointCloudInfo(const std::string& backendIdUtf8, PluginPointCloudInfo& out) const = 0;

	/// 1.2.0+：点云度量（UI 线程）
	virtual bool measurePointCloud(const std::string& backendIdUtf8, PluginPointCloudMeasure& out) const = 0;

	/// 1.2.0+：导出三角网格为 PLY（含 face；pathUtf8 为 UTF-8）
	virtual bool exportMeshToPly(
		const std::string& backendIdUtf8,
		const std::string& pathUtf8,
		std::string* outError = nullptr) const = 0;
};
