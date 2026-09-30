#ifndef CLOUDSIMHOST_PLUGINHOSTCONTEXT_H
#define CLOUDSIMHOST_PLUGINHOSTCONTEXT_H

/// @file PluginHostContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief IPluginHostContext 宿主实现（仅 Widget，不导出插件）

#include "IPluginHostContext.h"

#include "PluginContextImpls.h"
#include "cloudsim_host_global.h"

#include <QHash>
#include <QObject>
#include <functional>
#include <memory>
#include <vector>

class AiAssistantHostImpl;
class PluginGeometryHostImpl;
class PluginLabelingHostImpl;
class PluginPointCloudHostImpl;
class PluginRobotHostImpl;
class BackendDataBase;
class DocumentPage;
class IPluginMainWindowHost;
class PluginDocumentAdapter;
class PluginSceneBridgeAdapter;

/// IPluginHostContext 宿主实现（仅 Widget，不导出插件）
/// 域方法保留在本类供 ContextImpls/Host 内部调用，不再挂聚合虚表
class CLOUDSIM_HOST_EXPORT PluginHostContext : public QObject, public IPluginHostContext
{
	Q_OBJECT

public:
	explicit PluginHostContext(IPluginMainWindowHost* mainWindowHost, QObject* parent = nullptr);
	~PluginHostContext() override;

	void attachDocumentTabSignals();
	void refreshDocumentAdapters();

	unsigned int hostVersion() const override;
	QString applicationDirPath() const override;

	void logInfo(const QString& message) const override;
	void logWarn(const QString& message) const override;
	void logError(const QString& message) const override;

	bool useChinese() const override;
	void onLanguageChanged(std::function<void(bool useChinese)> callback) override;

	IPluginDocumentContext* documentContext() override;
	IPluginUiContext* uiContext() override;
	IPluginGeometryContext* geometryContext() override;
	IPluginAiContext* aiContext() override;
	IPluginRobotContext* robotContext() override;
	IPluginJobContext* jobContext() override;
	IPluginProjectContext* projectContext() override;

	// --- 以下为宿主内部/窄 ContextImpl 转发目标，非 IPluginHostContext 虚表 ---

	int documentCount() const;
	IPluginDocument* activeDocument();
	const IPluginDocument* activeDocument() const;
	IPluginDocument* documentAt(int index);
	const IPluginDocument* documentAt(int index) const;

	void onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback);
	void invokeOnUiThread(std::function<void()> fn);

	void enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
					std::function<void(bool threw, const QString& throwMessage)> onFinished);
	quint64 enqueueCancellableJob(const QString& title, PluginCancellableJobWorkFn work,
								  std::function<void(bool threw, const QString& throwMessage)> onFinished);
	bool cancelJob(quint64 jobId);
	IPluginDocument* documentById(const QString& documentId);
	const IPluginDocument* documentById(const QString& documentId) const;
	void onDocumentClosed(std::function<void(const QString& documentId)> callback);
	void invokeDocumentClosed(const QString& documentId);

	QDockWidget* registerDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area);
	QWidget* sidePanelTabParent() const;
	int registerSidePanelTab(const char* titleUtf8, QWidget* widget);
	void unregisterSidePanelTab(QWidget* widget);
	QMenu* registerMenuPath(const QStringList& path);
	QAction* registerAction(QMenu* menu, const QString& text, std::function<void()> handler);

	bool createPrimitiveMesh(const PluginPrimitiveMeshParams& params, const PluginPrimitiveMeshQuality& quality,
							 const PluginMeshCreateOptions& options, QString* outError,
							 QString* outBackendId = nullptr);

	bool booleanMesh(PluginMeshBooleanOp op, const std::string& targetBackendId, const std::string& toolBackendId,
					 const PluginBooleanMeshOptions& options, std::string* outResultBackendId, QString* outError);

	bool registerBackendType(const PluginBackendMeta& meta, QString* outError);
	bool registerTriangleMesh(const std::vector<float>& triangleSoup, const PluginMeshCreateOptions& options,
							  QString* outError);

	std::string importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud, std::string* outError);

	IPluginPointCloudHost* pointCloudHost();
	const IPluginPointCloudHost* pointCloudHost() const;

	IPluginRobotHost* robotHost();
	const IPluginRobotHost* robotHost() const;

	IPluginGeometryHost* geometryHost();
	const IPluginGeometryHost* geometryHost() const;

	void setSidePanelTabTitle(QWidget* widget, const char* titleUtf8);

	bool buildPrimitiveMeshSoup(const PluginPrimitiveMeshParams& params, const PluginPrimitiveMeshQuality& quality,
								const PluginMeshCreateOptions& placement, std::vector<float>& outWorldSoup,
								QString* outError);

	bool booleanMeshSoups(PluginMeshBooleanOp op, const std::vector<float>& targetWorldSoup,
						  const std::vector<float>& toolWorldSoup, const PluginBooleanMeshOptions& options,
						  std::string* outResultBackendId, QString* outError);

	bool booleanPrimitiveMeshes(PluginMeshBooleanOp op, const PluginPrimitiveMeshParams& targetParams,
								const PluginPrimitiveMeshQuality& targetQuality,
								const PluginMeshCreateOptions& targetPlacement,
								const PluginPrimitiveMeshParams& toolParams,
								const PluginPrimitiveMeshQuality& toolQuality,
								const PluginMeshCreateOptions& toolPlacement, const PluginBooleanMeshOptions& options,
								std::string* outResultBackendId, QString* outError);

	IAiAssistantHost* aiAssistantHost();
	const IAiAssistantHost* aiAssistantHost() const;

	bool captureActiveViewportPng(QByteArray& outPng, QString* outError = nullptr);

	bool resolveTrajectoryWorkpiece(QString& outBackendId, QString& outStepPath, QString* outError = nullptr);
	bool buildTrajectoryFeatureCatalogSlice(const QString& backendId, const QString& stepPathUtf8,
											const QString& userText, QByteArray& outFullCatalogUtf8,
											QByteArray& outSliceUtf8, QString* outError = nullptr);
	bool showAiFeatureCandidatePreview(const QByteArray& previewJsonUtf8, QString* outError = nullptr);
	void clearAiFeatureCandidatePreview();
	bool commitAiTrajectoryFeatures(const QByteArray& featurePlanJsonUtf8, QString* outSummary,
									QString* outError = nullptr);
	int proposeAndConfirmTrajectoryPlan(const QByteArray& planInUtf8, QByteArray& planOutUtf8,
										QString* outError = nullptr, bool showRetry = true);
	bool loadBoundTrajectoryPlanForAi(QByteArray& planOutUtf8, QString* outError = nullptr);
	bool reviseAiTrajectoryPlan(const QByteArray& planJsonUtf8, QString* outSummary, QString* outError = nullptr);

	IPluginLabelingHost* labelingHost();
	const IPluginLabelingHost* labelingHost() const;

	void setCentralAlternateWidget(QWidget* widget);
	void showCentralScene3D();
	void showCentralAlternate();
	bool isShowingCentralAlternate() const;
	void enterProcessFlowSideUi(QWidget* leftPanel, QWidget* rightPanel);
	void exitProcessFlowSideUi();
	void enterAlternateSideUi(QWidget* leftPanel, QWidget* rightPanel);
	void exitAlternateSideUi();

	void onProjectAboutToSave(std::function<void(const QString& documentId, QJsonObject& root)> callback);
	void onProjectLoaded(std::function<void(const QString& documentId, const QJsonObject& root)> callback);
	void invokeProjectAboutToSave(const QString& documentId, QJsonObject& root);
	void invokeProjectLoaded(const QString& documentId, const QJsonObject& root);

	void onParametricBodyHistoryChanged(
		std::function<void(const QString& documentId, const QString& backendId)> callback);
	void invokeParametricBodyHistoryChanged(const QString& documentId, const QString& backendId);

	void setProcessFlowAiBridge(IProcessFlowAiBridge* bridge);
	IProcessFlowAiBridge* processFlowAiBridge();
	const IProcessFlowAiBridge* processFlowAiBridge() const;

	bool embedActiveRenderWidget(QWidget* slot, QString* outError = nullptr);
	void restoreActiveRenderWidget();
	void setModeToolBar(QWidget* toolBar);

	void claimWorkspaceMode(const QString& modeId);
	void onWorkspaceModeClaimed(std::function<void(const QString& modeId)> callback);
	QString currentWorkspaceMode() const;

	void registerWorkspaceMode(const QString& modeId, const QString& titleZh, const QString& titleEn,
							   std::function<void()> enterFn);
	void returnToMainWorkspace();
	void enterWorkspaceMode(const QString& modeId);

	void markActiveDocumentModified();
	void clearActiveDocumentModified();
	bool isActiveDocumentModified() const;

	struct WorkspaceModeRegistration
	{
		QString modeId;
		QString titleZh;
		QString titleEn;
		std::function<void()> enterFn;
	};
	const std::vector<WorkspaceModeRegistration>& workspaceModes() const { return m_workspaceModes; }

	/// AI ActionPlan：树选中对象 id
	QString selectedBackendId() const;

	void notifyLanguageChanged();

	IPluginMainWindowHost* mainWindowHost() const { return m_mainWindowHost; }

	/// 插件 initialize 期间调用，用于侧栏页签稳定 objectName
	void beginPluginRegistration(const QString& pluginId);
	void endPluginRegistration();

	/// unload 前丢掉捕获插件 this 的回调/enterFn，避免后继插件 shutdown 踩悬空
	void prepareForPluginShutdown();

private:
	bool booleanSoupsAndRegister(const std::vector<float>& targetWorldSoup, const std::vector<float>& toolWorldSoup,
								 PluginMeshBooleanOp op, const PluginBooleanMeshOptions& options,
								 std::string* outResultBackendId, QString* outError);

	bool registerMeshFromSoup(std::vector<float> soup, const PluginMeshCreateOptions& options, QString* outError,
							  QString* outBackendId = nullptr);

	void ensureBuiltinMainWorkspaceMode();

	IPluginMainWindowHost* m_mainWindowHost = nullptr;
	std::unique_ptr<PluginPointCloudHostImpl> m_pointCloudHost;
	std::unique_ptr<PluginRobotHostImpl> m_robotHost;
	std::unique_ptr<PluginGeometryHostImpl> m_geometryHost;
	std::unique_ptr<PluginLabelingHostImpl> m_labelingHost;
	std::unique_ptr<AiAssistantHostImpl> m_aiHost;
	std::vector<std::function<void(bool useChinese)>> m_languageCallbacks;
	std::vector<std::unique_ptr<PluginDocumentAdapter>> m_documents;
	std::vector<std::function<void(IPluginDocument*)>> m_docChangeCallbacks;
	std::vector<std::function<void(const QString&)>> m_documentClosedCallbacks;
	std::vector<QDockWidget*> m_ownedDocks;
	std::vector<std::function<void(const QString&, QJsonObject&)>> m_projectSaveCallbacks;
	std::vector<std::function<void(const QString&, const QJsonObject&)>> m_projectLoadCallbacks;
	std::vector<std::function<void(const QString&, const QString&)>> m_parametricHistoryCallbacks;
	IProcessFlowAiBridge* m_processFlowAiBridge = nullptr;
	QString m_workspaceMode;
	std::vector<std::function<void(const QString&)>> m_workspaceModeCallbacks;
	std::vector<WorkspaceModeRegistration> m_workspaceModes;
	QString m_registeringPluginId;
	QHash<QString, int> m_pluginSidePanelTabSerial;

	/// 窄接口领域对象：仅转发回本宿主，随宿主同生命周期
	PluginDocumentContextImpl m_documentContextImpl;
	PluginUiContextImpl m_uiContextImpl;
	PluginGeometryContextImpl m_geometryContextImpl;
	PluginAiContextImpl m_aiContextImpl;
	PluginRobotContextImpl m_robotContextImpl;
	PluginJobContextImpl m_jobContextImpl;
	PluginProjectContextImpl m_projectContextImpl;
};

#endif // CLOUDSIMHOST_PLUGINHOSTCONTEXT_H
