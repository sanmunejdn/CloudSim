#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWPLUGIN_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWPLUGIN_H

/// @file ProcessFlowPlugin.h
/// @brief 工艺流程仿真插件

#include "ICloudSimPlugin.h"
#include "ProcessFlowAiBridge.h"
#include "ProcessFlowNodeProps.h"
#include "sim/SimStatistics.h"

#include <QHash>
#include <QObject>
#include <QPointer>
#include <QVector>
#include <memory>

class ProcessFlowCanvasWidget;
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
	~ProcessFlowPlugin() override;

	QString pluginId() const override;
	QString displayName() const override;
	bool initialize(IPluginHostContext* host) override;
	void shutdown() override;

	bool ensureProcessFlowForAi(QString* outError);
	ProcessFlowCanvasWidget* activeCanvasForAi() const;
	void syncJobSetPanelFromCanvas();
	void applyAiSimStatistics(const SimStatistics& stats);
	void applyAiCompareRows(const QVector<PolicyCompareRow>& rows);

private:
	void registerMenus();
	void applyLanguage();
	void enterProcessFlow();
	void exitProcessFlow();
	void softExitProcessFlow();
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
	std::unique_ptr<ProcessFlowAiBridge> m_aiBridge;
	bool m_inProcessFlow = false;
	bool m_flowDirty = false;
};

#endif
