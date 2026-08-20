#ifndef GEOMETRYPLUGIN_GEOMETRYPLUGIN_H
#define GEOMETRYPLUGIN_GEOMETRYPLUGIN_H

/// @file GeometryPlugin.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief GeometryPlugin 接口

#include "ICloudSimPlugin.h"

#include <QObject>

class GeometryDockWidget;
class IPluginHostContext;
class QMenu;

class GeometryPlugin : public QObject, public ICloudSimPlugin
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

	IPluginHostContext* m_host = nullptr;
	GeometryDockWidget* m_dockWidget = nullptr;
	QMenu* m_geometryMenu = nullptr;
};

#endif // GEOMETRYPLUGIN_GEOMETRYPLUGIN_H
