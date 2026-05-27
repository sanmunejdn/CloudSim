#pragma once

#include "ICloudSimPlugin.h"

#include <QObject>

class IPluginHostContext;
class QWidget;

class PlcCommPlugin : public QObject, public ICloudSimPlugin
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
	void applyLanguage();

	IPluginHostContext* host_ = nullptr;
	QWidget* panel_ = nullptr;
};
