#ifndef CLOUDSIMPLUGINHOST_PLUGINDELEGATEDBACKEND_H
#define CLOUDSIMPLUGINHOST_PLUGINDELEGATEDBACKEND_H

/// @file PluginDelegatedBackend.h
/// @brief IPluginBackendObject 适配为 BackendDataBase（BackendRegistry）

#include "BackendDataBase.h"
#include "PluginBackendMeta.h"

#include <memory>

/// IPluginBackendObject 适配为 BackendDataBase（BackendRegistry）
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

#endif // CLOUDSIMPLUGINHOST_PLUGINDELEGATEDBACKEND_H
