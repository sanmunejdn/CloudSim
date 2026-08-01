#ifndef CLOUDSIMPLUGINHOST_PLUGINHOSTCONTEXT_H
#define CLOUDSIMPLUGINHOST_PLUGINHOSTCONTEXT_H

/// @file PluginHostContext.h
/// @brief IPluginHostContext 宿主实现（仅 Widget，不导出插件）

#include "IPluginHostContext.h"

#include <QHash>
#include <QObject>
#include <functional>
#include <memory>
#include <vector>

class AiAssistantHostImpl;
class PluginGeometryHostImpl;
class PluginLabelingHostImpl;
class PluginPointCloudHostImpl;
class BackendDataBase;
class DocumentPage;
class IPluginMainWindowHost;
class PluginDocumentAdapter;
class PluginSceneBridgeAdapter;

/// IPluginHostContext 宿主实现（仅 Widget，不导出插件）
class PluginHostContext : public QObject, public IPluginHostContext
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

	int documentCount() const override;
	IPluginDocument* activeDocument() override;
	const IPluginDocument* activeDocument() const override;
	IPluginDocument* documentAt(int index) override;
	const IPluginDocument* documentAt(int index) const override;

	void onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback) override;
	void invokeOnUiThread(std::function<void()> fn) override;

	void enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
					std::function<void(bool threw, const QString& throwMessage)> onFinished) override;

	QDockWidget* registerDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area) override;
	QWidget* sidePanelTabParent() const override;
	int registerSidePanelTab(const char* titleUtf8, QWidget* widget) override;
	void unregisterSidePanelTab(QWidget* widget) override;
	QMenu* registerMenuPath(const QStringList& path) override;
	QAction* registerAction(QMenu* menu, const QString& text, std::function<void()> handler) override;

	bool createPrimitiveMesh(const PluginPrimitiveMeshParams& params, const PluginPrimitiveMeshQuality& quality,
							 const PluginMeshCreateOptions& options, QString* outError,
							 QString* outBackendId = nullptr) override;

	bool booleanMesh(PluginMeshBooleanOp op, const std::string& targetBackendId, const std::string& toolBackendId,
					 const PluginBooleanMeshOptions& options, std::string* outResultBackendId,
					 QString* outError) override;

	bool registerBackendType(const PluginBackendMeta& meta, QString* outError) override;
	bool registerTriangleMesh(const std::vector<float>& triangleSoup, const PluginMeshCreateOptions& options,
							  QString* outError) override;

	std::string importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud,
											 std::string* outError) override;

	IPluginPointCloudHost* pointCloudHost() override;
	const IPluginPointCloudHost* pointCloudHost() const override;

	IPluginGeometryHost* geometryHost() override;
	const IPluginGeometryHost* geometryHost() const override;

	bool useChinese() const override;
	void onLanguageChanged(std::function<void(bool useChinese)> callback) override;
	void setSidePanelTabTitle(QWidget* widget, const char* titleUtf8) override;

	bool buildPrimitiveMeshSoup(const PluginPrimitiveMeshParams& params, const PluginPrimitiveMeshQuality& quality,
								const PluginMeshCreateOptions& placement, std::vector<float>& outWorldSoup,
								QString* outError) override;

	bool booleanMeshSoups(PluginMeshBooleanOp op, const std::vector<float>& targetWorldSoup,
						  const std::vector<float>& toolWorldSoup, const PluginBooleanMeshOptions& options,
						  std::string* outResultBackendId, QString* outError) override;

	bool booleanPrimitiveMeshes(PluginMeshBooleanOp op, const PluginPrimitiveMeshParams& targetParams,
								const PluginPrimitiveMeshQuality& targetQuality,
								const PluginMeshCreateOptions& targetPlacement,
								const PluginPrimitiveMeshParams& toolParams,
								const PluginPrimitiveMeshQuality& toolQuality,
								const PluginMeshCreateOptions& toolPlacement, const PluginBooleanMeshOptions& options,
								std::string* outResultBackendId, QString* outError) override;

	IAiAssistantHost* aiAssistantHost() override;
	const IAiAssistantHost* aiAssistantHost() const override;

	bool captureActiveViewportPng(QByteArray& outPng, QString* outError = nullptr) override;

	bool resolveTrajectoryWorkpiece(QString& outBackendId, QString& outStepPath, QString* outError = nullptr) override;
	bool buildTrajectoryFeatureCatalogSlice(const QString& backendId, const QString& stepPathUtf8,
											const QString& userText, QByteArray& outFullCatalogUtf8,
											QByteArray& outSliceUtf8, QString* outError = nullptr) override;
	bool showAiFeatureCandidatePreview(const QByteArray& previewJsonUtf8, QString* outError = nullptr) override;
	void clearAiFeatureCandidatePreview() override;
	bool commitAiTrajectoryFeatures(const QByteArray& featurePlanJsonUtf8, QString* outSummary,
									QString* outError = nullptr) override;

	IPluginLabelingHost* labelingHost() override;
	const IPluginLabelingHost* labelingHost() const override;

	void setCentralAlternateWidget(QWidget* widget) override;
	void showCentralScene3D() override;
	void showCentralAlternate() override;
	bool isShowingCentralAlternate() const override;
	void enterProcessFlowSideUi(QWidget* leftPanel, QWidget* rightPanel) override;
	void exitProcessFlowSideUi() override;
	void enterAlternateSideUi(QWidget* leftPanel, QWidget* rightPanel) override;
	void exitAlternateSideUi() override;

	void onProjectAboutToSave(std::function<void(const QString& documentId, QJsonObject& root)> callback) override;
	void onProjectLoaded(std::function<void(const QString& documentId, const QJsonObject& root)> callback) override;
	void invokeProjectAboutToSave(const QString& documentId, QJsonObject& root);
	void invokeProjectLoaded(const QString& documentId, const QJsonObject& root);

	void onParametricBodyHistoryChanged(
		std::function<void(const QString& documentId, const QString& backendId)> callback) override;
	void invokeParametricBodyHistoryChanged(const QString& documentId, const QString& backendId);

	void setProcessFlowAiBridge(IProcessFlowAiBridge* bridge) override;
	IProcessFlowAiBridge* processFlowAiBridge() override;
	const IProcessFlowAiBridge* processFlowAiBridge() const override;

	bool embedActiveRenderWidget(QWidget* slot, QString* outError = nullptr) override;
	void restoreActiveRenderWidget() override;
	void setModeToolBar(QWidget* toolBar) override;

	void claimWorkspaceMode(const QString& modeId) override;
	void onWorkspaceModeClaimed(std::function<void(const QString& modeId)> callback) override;
	QString currentWorkspaceMode() const override;

	void registerWorkspaceMode(const QString& modeId, const QString& titleZh, const QString& titleEn,
							   std::function<void()> enterFn) override;
	void returnToMainWorkspace() override;
	void enterWorkspaceMode(const QString& modeId) override;

	void markActiveDocumentModified() override;
	void clearActiveDocumentModified() override;
	bool isActiveDocumentModified() const override;

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

private:
	bool booleanSoupsAndRegister(const std::vector<float>& targetWorldSoup, const std::vector<float>& toolWorldSoup,
								 PluginMeshBooleanOp op, const PluginBooleanMeshOptions& options,
								 std::string* outResultBackendId, QString* outError);

	bool registerMeshFromSoup(std::vector<float> soup, const PluginMeshCreateOptions& options, QString* outError,
							  QString* outBackendId = nullptr);

	void ensureBuiltinMainWorkspaceMode();

	IPluginMainWindowHost* m_mainWindowHost = nullptr;
	std::unique_ptr<PluginPointCloudHostImpl> m_pointCloudHost;
	std::unique_ptr<PluginGeometryHostImpl> m_geometryHost;
	std::unique_ptr<PluginLabelingHostImpl> m_labelingHost;
	std::unique_ptr<AiAssistantHostImpl> m_aiHost;
	std::vector<std::function<void(bool useChinese)>> m_languageCallbacks;
	std::vector<std::unique_ptr<PluginDocumentAdapter>> m_documents;
	std::vector<std::function<void(IPluginDocument*)>> m_docChangeCallbacks;
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
};

#endif // CLOUDSIMPLUGINHOST_PLUGINHOSTCONTEXT_H
