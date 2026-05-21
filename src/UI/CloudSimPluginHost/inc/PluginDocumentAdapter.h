#pragma once

#include "IPluginDocument.h"

#include <memory>
#include <string>

class DocumentPage;
class PluginSceneBridgeAdapter;

class PluginDocumentAdapter : public IPluginDocument
{
public:
	explicit PluginDocumentAdapter(DocumentPage* page);

	std::string documentLabel() const override;
	std::size_t backendObjectCount() const override;
	std::vector<std::string> backendIds() const override;
	bool containsBackend(const std::string& backendId) const override;
	std::string backendDisplayName(const std::string& backendId) const override;
	std::string backendClassName(const std::string& backendId) const override;

	IPluginSceneBridge* sceneBridge() override;
	const IPluginSceneBridge* sceneBridge() const override;

	DocumentPage* documentPage() const { return m_page; }

private:
	DocumentPage* m_page = nullptr;
	std::unique_ptr<PluginSceneBridgeAdapter> m_sceneBridge;
};
