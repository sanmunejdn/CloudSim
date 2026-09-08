#ifndef CLOUDSIMPLUGINHOST_PLUGINDELEGATEDBACKEND_H
#define CLOUDSIMPLUGINHOST_PLUGINDELEGATEDBACKEND_H

/// @file PluginDelegatedBackend.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief IPluginBackendObject 适配为 BackendDataBase（BackendRegistry）

#include "BackendDataBase.h"
#include "PluginBackendMeta.h"

#include <memory>

struct PluginDelegatedBackendOptions
{
	bool supportsTransform = true;
	bool supportsVisibility = true;
	bool usePropertyBindings = false;
};

/// IPluginBackendObject 适配为 BackendDataBase（BackendRegistry）
class PluginDelegatedBackend : public BackendDataBase
{
public:
	PluginDelegatedBackend(std::shared_ptr<IPluginBackendObject> delegate, PluginDelegatedBackendOptions options);

	std::string className() const override;
	bool hasGeometry() const override;
	BackendBoundingBox geometryBounds() const override;
	std::size_t geometryElementCount() const override;
	void clearGeometry() override;

	bool hasPoseProperty() const override;
	bool hasRotationProperty() const override;

	nlohmann::json snapshotPropertyRows(const BackendDataManager* mgr = nullptr) const override;
	bool applyPropertyChange(const std::string& key, const std::string& value, std::string* errMsg,
							 const BackendDataManager* mgr = nullptr) override;

	std::shared_ptr<IPluginBackendObject> delegate() const { return m_delegate; }

private:
	std::shared_ptr<IPluginBackendObject> m_delegate;
	PluginDelegatedBackendOptions m_options;
};

#endif // CLOUDSIMPLUGINHOST_PLUGINDELEGATEDBACKEND_H
