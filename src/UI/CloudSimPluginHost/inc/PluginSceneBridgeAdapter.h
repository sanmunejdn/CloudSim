#pragma once

#include "IPluginSceneBridge.h"

class DocumentPage;
class IBackendSceneBridge;

class PluginSceneBridgeAdapter : public IPluginSceneBridge
{
public:
	explicit PluginSceneBridgeAdapter(DocumentPage* page);

	bool setBackendRootWorldMatrixColumnMajor(const std::string& backendId,
		const std::array<double, 16>& columnMajor4x4) override;
	bool getBackendRootWorldMatrixColumnMajor(const std::string& backendId,
		std::array<double, 16>& outColumnMajor4x4) const override;

	void setBackendObjectVisible(const std::string& backendId, bool visible) override;
	void removeBackendObjectVisual(const std::string& backendId) override;
	bool hasBackendObjectBranch(const std::string& backendId) const override;

private:
	IBackendSceneBridge* bridge() const;

	DocumentPage* m_page = nullptr;
};
