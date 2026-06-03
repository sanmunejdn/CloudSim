#pragma once

#include "IPluginDocument.h"

#include <memory>
#include <string>

namespace cloudsim::host {
class DocumentHost;
}

class PluginSceneBridgeAdapter;

class PluginDocumentAdapter : public IPluginDocument
{
public:
	explicit PluginDocumentAdapter(cloudsim::host::DocumentHost* host);

	std::string documentLabel() const override;
	std::size_t backendObjectCount() const override;
	std::vector<std::string> backendIds() const override;
	bool containsBackend(const std::string& backendId) const override;
	std::string backendDisplayName(const std::string& backendId) const override;
	std::string backendClassName(const std::string& backendId) const override;
	std::string documentId() const override;
	bool removeBackendObject(const std::string& backendIdUtf8, std::string* outError) override;

	IPluginSceneBridge* sceneBridge() override;
	const IPluginSceneBridge* sceneBridge() const override;

	bool queryPointCloudInfo(const std::string& backendIdUtf8, PluginPointCloudInfo& out) const override;
	bool measurePointCloud(const std::string& backendIdUtf8, PluginPointCloudMeasure& out) const override;
	bool exportMeshToPly(
		const std::string& backendIdUtf8,
		const std::string& pathUtf8,
		std::string* outError = nullptr) const override;

	cloudsim::host::DocumentHost* documentHost() const { return m_host; }

private:
	cloudsim::host::DocumentHost* m_host = nullptr;
	std::unique_ptr<PluginSceneBridgeAdapter> m_sceneBridge;
};
