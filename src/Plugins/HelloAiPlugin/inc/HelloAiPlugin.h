#pragma once

#include "ICloudSimAiPlugin.h"
#include "ICloudSimPlugin.h"

#include <QObject>

class HelloAiPlugin : public QObject, public ICloudSimPlugin, public ICloudSimAiPlugin
{
	Q_OBJECT
	Q_PLUGIN_METADATA(IID "com.cloudsim.ICloudSimPlugin/1.0")
	Q_INTERFACES(ICloudSimPlugin ICloudSimAiPlugin)

public:
	HelloAiPlugin() = default;

	QString pluginId() const override;
	QString displayName() const override;
	bool initialize(IPluginHostContext* host) override;
	void shutdown() override;

	QString aiPluginId() const override;
	bool initializeAi(IPluginHostContext* host, IAiAssistantHost* aiHost) override;
	void shutdownAi() override;

private:
	IPluginHostContext* m_host = nullptr;
	IAiAssistantHost* m_aiHost = nullptr;
};
