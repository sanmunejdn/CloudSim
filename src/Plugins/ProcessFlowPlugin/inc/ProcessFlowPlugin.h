#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWPLUGIN_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWPLUGIN_H

/// @file ProcessFlowPlugin.h
/// @brief 工艺流程仿真插件

#include "ICloudSimPlugin.h"
#include "ProcessFlowNodeProps.h"

#include <QHash>
#include <QObject>
#include <QPointer>

class ProcessFlowPageWidget;
class ProcessFlowPaletteWidget;
class ProcessFlowSimController;
class ProcessFlowSimSideWidget;
class QAction;
class QColor;
class QJsonObject;
class QMenu;

class ProcessFlowPlugin : public QObject, public ICloudSimPlugin
{
	Q_OBJECT
	Q_PLUGIN_METADATA(IID "com.cloudsim.ICloudSimPlugin/1.0")
	Q_INTERFACES(ICloudSimPlugin)

public:
	ProcessFlowPlugin() = default;

	QString pluginId() const override;
	QString displayName() const override;
	bool initialize(IPluginHostContext* host) override;
	void shutdown() override;

private:
	void registerMenus();
	void applyLanguage();
	void enterProcessFlow();
	void exitProcessFlow();
	ProcessFlowPageWidget* ensurePageForDocument(const QString& documentId);
	ProcessFlowPageWidget* ensurePageForActiveDocument();
	void addNodeToActiveCanvas(const QString& kind, const QString& title, const QString& subtitle, const QColor& color);
	void bindCanvasSelection(ProcessFlowPageWidget* page);
	void bindSimUi();
	void runSimulation();
	void exportSimJson();
	void exportSimCsv();
	void onProjectAboutToSave(const QString& documentId, QJsonObject& root);
	void onProjectLoaded(const QString& documentId, const QJsonObject& root);

	IPluginHostContext* m_host = nullptr;
	QMenu* m_menu = nullptr;
	QAction* m_enterAction = nullptr;
	QAction* m_exitAction = nullptr;
	QAction* m_runAction = nullptr;
	QAction* m_stopAction = nullptr;
	QPointer<ProcessFlowPaletteWidget> m_palette;
	QPointer<ProcessFlowSimSideWidget> m_simSide;
	QHash<QString, QPointer<ProcessFlowPageWidget>> m_pagesByDocId;
	ProcessFlowSimController* m_sim = nullptr;
	bool m_inProcessFlow = false;
};

#endif
