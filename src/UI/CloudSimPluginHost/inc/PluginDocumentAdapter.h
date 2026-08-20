#ifndef CLOUDSIMPLUGINHOST_PLUGINDOCUMENTADAPTER_H
#define CLOUDSIMPLUGINHOST_PLUGINDOCUMENTADAPTER_H

/// @file PluginDocumentAdapter.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PluginDocumentAdapter 接口

#include "IPluginDocument.h"

#include <memory>
#include <string>

namespace cloudsim::host
{
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
	bool exportMeshToPly(const std::string& backendIdUtf8, const std::string& pathUtf8,
						 std::string* outError = nullptr) const override;

	struct WorldPoseMm
	{
		double xMm = 0.0;
		double yMm = 0.0;
		double zMm = 0.0;
		double rxDeg = 0.0;
		double ryDeg = 0.0;
		double rzDeg = 0.0;
	};
	bool getWorldPoseMm(const std::string& backendIdUtf8, WorldPoseMm* out) const;
	bool applyWorldPoseMm(const std::string& backendIdUtf8, const WorldPoseMm& pose, std::string* outError = nullptr);

	cloudsim::host::DocumentHost* documentHost() const { return m_host; }

private:
	cloudsim::host::DocumentHost* m_host = nullptr;
	std::unique_ptr<PluginSceneBridgeAdapter> m_sceneBridge;
};

#endif // CLOUDSIMPLUGINHOST_PLUGINDOCUMENTADAPTER_H
