#ifndef CLOUDSIMPLUGINHOST_IPLUGINMAINWINDOWHOST_H
#define CLOUDSIMPLUGINHOST_IPLUGINMAINWINDOWHOST_H

/// @file IPluginMainWindowHost.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 与 IPluginHostContext::PluginJobProgressFn 同型，避免 Widget/CloudSim 依赖 PluginSDK 头

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

namespace cloudsim::host
{
class DocumentHost;
}

/// Widget 侧能力，供 Host 内 PluginHostContext 使用（不链接 Widget 具体类型）
class IPluginMainWindowHost
{
public:
	virtual ~IPluginMainWindowHost() = default;

	/// Widget 树当前选中后端 id（AI/插件对齐 Dock「当前对象」）
	virtual QString selectedBackendId() const = 0;

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

	/// 1.17.0+：当前文档中央 alternate / 工艺流程右侧模式
	virtual void setCentralAlternateWidget(QWidget* widget) = 0;
	virtual void showCentralScene3D() = 0;
	virtual void showCentralAlternate() = 0;
	virtual bool isShowingCentralAlternate() const = 0;
	virtual void enterProcessFlowSideUi(QWidget* leftPanel, QWidget* rightPanel) = 0;
	virtual void exitProcessFlowSideUi() = 0;

	/// 1.34.0+：通用侧栏别名（实现与 enterProcessFlowSideUi 相同）
	virtual void enterAlternateSideUi(QWidget* leftPanel, QWidget* rightPanel) = 0;
	virtual void exitAlternateSideUi() = 0;

	/// 1.21.0+：活动文档 OsgWidget 嵌入/还原
	virtual bool embedActiveRenderWidget(QWidget* slot, QString* outError = nullptr) = 0;
	virtual void restoreActiveRenderWidget() = 0;

	/// 1.22.0+：菜单下模式工具条（nullptr 清除）
	virtual void setModeToolBar(QWidget* toolBar) = 0;

	/// 1.35.0+：插件 registerWorkspaceMode 后刷新顶栏分段
	virtual void notifyWorkspaceModesChanged() = 0;

	/// 1.36.0+：活动文档未保存 *
	virtual void markActiveDocumentModified() = 0;
	virtual void clearActiveDocumentModified() = 0;
	virtual bool isActiveDocumentModified() const = 0;

	/// 1.51.0+：模态确认离散策略/参数/管线；返回 1=接受 0=取消 2=重新识别
	virtual int proposeAndConfirmTrajectoryPlanForAi(const std::string& planInUtf8, std::string* planOutUtf8,
													 QString* outError, bool showRetry = true) = 0;
	virtual bool loadBoundTrajectoryPlanForAi(std::string* planOutUtf8, QString* outError) = 0;
	virtual bool reviseAiTrajectoryPlanForAi(const std::string& planJsonUtf8, QString* outSummary,
											 QString* outError) = 0;

	using PluginJobCanceledFn = std::function<bool()>;
	virtual quint64 enqueueCancellableBackgroundJob(
		const QString& title,
		std::function<void(const PluginJobProgressFn& progress, const PluginJobCanceledFn& canceled)> work,
		std::function<void(bool threw, const QString& message)> onFinished) = 0;
	virtual bool cancelBackgroundJob(quint64 jobId) = 0;
};

#endif // CLOUDSIMPLUGINHOST_IPLUGINMAINWINDOWHOST_H
