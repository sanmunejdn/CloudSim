#pragma once

#include "IPluginSceneBridge.h"

namespace cloudsim::host {
class DocumentHost;
}

class IBackendSceneBridge;

class PluginSceneBridgeAdapter : public IPluginSceneBridge
{
public:
	explicit PluginSceneBridgeAdapter(cloudsim::host::DocumentHost* host);

	bool setBackendRootWorldMatrixColumnMajor(const std::string& backendId,
		const std::array<double, 16>& columnMajor4x4) override;
	bool getBackendRootWorldMatrixColumnMajor(const std::string& backendId,
		std::array<double, 16>& outColumnMajor4x4) const override;
	void setBackendObjectVisible(const std::string& backendId, bool visible) override;
	void removeBackendObjectVisual(const std::string& backendId) override;
	bool hasBackendObjectBranch(const std::string& backendId) const override;

	IBackendSceneBridge* bridge() const;

private:
	cloudsim::host::DocumentHost* m_host = nullptr;
};
