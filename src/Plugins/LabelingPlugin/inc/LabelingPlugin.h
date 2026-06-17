#pragma once

#include "ICloudSimPlugin.h"

#include <QObject>

class IPluginHostContext;
class LabelingAnnotWidget;
class LabelingTrainWidget;
class QTabWidget;
class QMenu;
class QAction;

class LabelingPlugin : public QObject, public ICloudSimPlugin
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

	IPluginHostContext* m_host = nullptr;
	QTabWidget* m_tabWidget = nullptr;
	LabelingAnnotWidget* m_annotWidget = nullptr;
	LabelingTrainWidget* m_trainWidget = nullptr;
	QMenu* m_labelingMenu = nullptr;
	QAction* m_openAnnotAction = nullptr;
	QAction* m_openTrainAction = nullptr;
};
