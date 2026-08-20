#ifndef POINTCLOUDPLUGIN_POINTCLOUDPLUGIN_H
#define POINTCLOUDPLUGIN_POINTCLOUDPLUGIN_H

/// @file PointCloudPlugin.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PointCloudPlugin 接口

#include "ICloudSimPlugin.h"

#include <QObject>

class QAction;
class PointCloudDockWidget;
class TubularGrindingDockWidget;
class IPluginHostContext;
class QMenu;

class PointCloudPlugin : public QObject, public ICloudSimPlugin
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
	void registerMenus();
	void applyLanguage();
	void importPointCloud();
	void downsampleVoxelOnSelection();
	void reconstructPoissonOnSelection();
	void exportMeshOnSelection();
	void simplifyMeshOnSelection();
	void smoothMeshOnSelection();
	void surfaceReconstructOnSelection();

	IPluginHostContext* m_host = nullptr;
	PointCloudDockWidget* m_dockWidget = nullptr;
	TubularGrindingDockWidget* m_featureBuildWidget = nullptr;
	QMenu* m_pointCloudMenu = nullptr;
	QAction* m_importAction = nullptr;
	QAction* m_downsampleAction = nullptr;
	QAction* m_reconstructAction = nullptr;
	QAction* m_exportMeshAction = nullptr;
	QAction* m_refreshAction = nullptr;
	QAction* m_simplifyAction = nullptr;
	QAction* m_smoothAction = nullptr;
	QAction* m_surfaceReconstructAction = nullptr;
};

#endif // POINTCLOUDPLUGIN_POINTCLOUDPLUGIN_H
