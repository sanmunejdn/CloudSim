#pragma once

#include "ICloudSimPlugin.h"

#include <QObject>

class HelloDockWidget;
class IPluginHostContext;

class HelloPlugin : public QObject, public ICloudSimPlugin
{
	Q_OBJECT
	Q_PLUGIN_METADATA(IID "com.cloudsim.ICloudSimPlugin/1.0")
	Q_INTERFACES(ICloudSimPlugin)

public:
	QString pluginId() const override;
	QString displayName() const override;
	bool initialize(IPluginHostContext* host) override;
	void shutdown() override;

private:
	void insertTestBox();

	IPluginHostContext* m_host = nullptr;
	HelloDockWidget* m_dockWidget = nullptr;
};
