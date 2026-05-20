#pragma once

#include "BackendDataBase.h"
#include "PluginBackendMeta.h"

#include <memory>

/// Adapts \ref IPluginBackendObject to \ref BackendDataBase for \c BackendRegistry (Phase 2).
class PluginDelegatedBackend : public BackendDataBase
{
public:
	explicit PluginDelegatedBackend(std::shared_ptr<IPluginBackendObject> delegate);

	std::string className() const override;
	bool hasGeometry() const override;
	BackendBoundingBox geometryBounds() const override;
	std::size_t geometryElementCount() const override;
	void clearGeometry() override;

	nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const override;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
		const BackendDataManager* mgr = nullptr) override;

	std::shared_ptr<IPluginBackendObject> delegate() const { return m_delegate; }

private:
	std::shared_ptr<IPluginBackendObject> m_delegate;
};
