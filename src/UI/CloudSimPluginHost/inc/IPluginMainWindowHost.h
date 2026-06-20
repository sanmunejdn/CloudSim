#pragma once

#include <QDockWidget>
#include <QString>
#include <functional>
#include <string>

/// 与 IPluginHostContext::PluginJobProgressFn 同型，避免 Widget/CloudSim 依赖 PluginSDK 头
using PluginJobProgressFn = std::function<void(double fraction, const QString& message)>;

class QMenu;
class QMenuBar;
class QObject;
class QTabWidget;
class QTreeWidget;
class QWidget;

namespace cloudsim::host {
class DocumentHost;
}

/// Widget 侧能力，供 Host 内 PluginHostContext 使用（不链接 Widget 具体类型）
class IPluginMainWindowHost
{
public:
	virtual ~IPluginMainWindowHost() = default;

	virtual QTabWidget* documentTabs() = 0;
	virtual int documentTabCount() const = 0;
	virtual cloudsim::host::DocumentHost* currentDocumentHost() = 0;
	virtual cloudsim::host::DocumentHost* documentHostAt(int tabIndex) = 0;
	virtual QTabWidget* rightPanelTabs() = 0;
	virtual int addPluginSidePanelTab(const QString& title, QWidget* widget) = 0;
	virtual void removePluginSidePanelTab(QWidget* widget) = 0;
	virtual void setPluginSidePanelTabTitle(QWidget* widget, const QString& title) = 0;
	virtual QMenuBar* menuBar() = 0;
	virtual void focusBackendInTree(const std::string& backendId) = 0;
	virtual void focusBackendInTreeAfterImport(const QString& backendId) = 0;
	virtual bool resolveTrajectoryWorkpieceForAi(QString* outBackendId, QString* outStepPath) = 0;
	virtual bool showAiFeatureCandidatePreviewForAi(const std::string& previewJsonUtf8, QString* outError) = 0;
	virtual void clearAiFeatureCandidatePreviewForAi() = 0;
	virtual bool commitAiTrajectoryFeaturesForAi(const std::string& featurePlanJsonUtf8, QString* outSummary,
		QString* outError) = 0;
	virtual bool useChinese() const = 0;
	virtual void appendRunInfo(const QString& message) = 0;
	virtual class QStatusBar* statusBar() = 0;
	virtual void enqueueBackgroundJob(const QString& title,
		std::function<void(const PluginJobProgressFn& progress)> work,
		std::function<void(bool threw, const QString& message)> onFinished) = 0;
	virtual QWidget* mainWindowWidget() = 0;
	virtual QObject* pluginActionParent() = 0;
	virtual QDockWidget* addPluginDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area) = 0;
};
