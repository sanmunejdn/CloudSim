#ifndef POINTNETPLUGIN_POINTNETPLUGIN_H
#define POINTNETPLUGIN_POINTNETPLUGIN_H

/// @file PointNetPlugin.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PointNet++ 插件主类：注册 pointnet.classify / pointnet.segment 两个 AI 域

#include "ICloudSimAiPlugin.h"
#include "ICloudSimPlugin.h"

#include <QObject>
#include <memory>

class PointNetInference;
class PointNetClassifyDomainHandler;
class PointNetSegmentDomainHandler;

/// PointNet++ 插件主类：注册 pointnet.classify / pointnet.segment 两个 AI 域
class PointNetPlugin : public QObject, public ICloudSimPlugin, public ICloudSimAiPlugin
{
	Q_OBJECT
	Q_PLUGIN_METADATA(IID "com.cloudsim.ICloudSimPlugin/1.0")
	Q_INTERFACES(ICloudSimPlugin ICloudSimAiPlugin)

public:
	PointNetPlugin();
	~PointNetPlugin() override;

	// ICloudSimPlugin
	QString pluginId() const override;
	QString displayName() const override;
	bool initialize(IPluginHostContext* host) override;
	void shutdown() override;

	// ICloudSimAiPlugin
	QString aiPluginId() const override;
	bool initializeAi(IPluginHostContext* host, IAiAssistantHost* aiHost) override;
	void shutdownAi() override;

private:
	/// 加载 pointnet_config.json 配置
	bool loadConfig(QString* err = nullptr);

	IPluginHostContext* m_host = nullptr;
	IAiAssistantHost* m_aiHost = nullptr;

	std::unique_ptr<PointNetInference> m_inference;
	std::unique_ptr<PointNetClassifyDomainHandler> m_clsHandler;
	std::unique_ptr<PointNetSegmentDomainHandler> m_segHandler;
};

#endif // POINTNETPLUGIN_POINTNETPLUGIN_H
