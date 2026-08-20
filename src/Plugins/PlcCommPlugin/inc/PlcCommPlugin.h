#ifndef PLCCOMMPLUGIN_PLCCOMMPLUGIN_H
#define PLCCOMMPLUGIN_PLCCOMMPLUGIN_H

/// @file PlcCommPlugin.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PlcCommPlugin 接口

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

#endif // PLCCOMMPLUGIN_PLCCOMMPLUGIN_H
