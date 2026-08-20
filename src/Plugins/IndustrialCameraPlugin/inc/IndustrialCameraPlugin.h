#ifndef INDUSTRIALCAMERAPLUGIN_INDUSTRIALCAMERAPLUGIN_H
#define INDUSTRIALCAMERAPLUGIN_INDUSTRIALCAMERAPLUGIN_H

/// @file IndustrialCameraPlugin.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 工业相机插件入口

#include "ICloudSimPlugin.h"

#include <QObject>

class IPluginHostContext;
class QWidget;

class IndustrialCameraPlugin : public QObject, public ICloudSimPlugin
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

#endif // INDUSTRIALCAMERAPLUGIN_INDUSTRIALCAMERAPLUGIN_H
