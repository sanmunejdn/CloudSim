#pragma once

#include "ICloudSimPlugin.h"

#include <QObject>

class QAction;
class PointCloudDockWidget;
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

	IPluginHostContext* m_host = nullptr;
	PointCloudDockWidget* m_dockWidget = nullptr;
	QMenu* m_pointCloudMenu = nullptr;
	QAction* m_importAction = nullptr;
	QAction* m_downsampleAction = nullptr;
	QAction* m_reconstructAction = nullptr;
	QAction* m_exportMeshAction = nullptr;
	QAction* m_refreshAction = nullptr;
};
