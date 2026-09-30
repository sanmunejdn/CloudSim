#ifndef CLOUDSIMHOST_PLUGINCONTEXTIMPLS_H
#define CLOUDSIMHOST_PLUGINCONTEXTIMPLS_H

/// @file PluginContextImpls.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 7 个窄接口的宿主实现：转发到 PluginHostContext（仅 Host DLL 内部使用）
/// @note 业务逻辑仍在聚合宿主；若改为聚合→窄转发会递归，故保持此方向

#include "IPluginAiContext.h"
#include "IPluginDocumentContext.h"
#include "IPluginGeometryContext.h"
#include "IPluginJobContext.h"
#include "IPluginProjectContext.h"
#include "IPluginRobotContext.h"
#include "IPluginUiContext.h"

class PluginHostContext;

class PluginDocumentContextImpl : public IPluginDocumentContext
{
public:
	explicit PluginDocumentContextImpl(PluginHostContext& host) : m_host(host) {}

	unsigned int contextVersion() const override { return 0x00010000; }

	int documentCount() const override;
	IPluginDocument* activeDocument() override;
	const IPluginDocument* activeDocument() const override;
	IPluginDocument* documentAt(int index) override;
	const IPluginDocument* documentAt(int index) const override;
	IPluginDocument* documentById(const QString& documentId) override;
	const IPluginDocument* documentById(const QString& documentId) const override;
	void onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback) override;
	void onDocumentClosed(std::function<void(const QString& documentId)> callback) override;
	void markActiveDocumentModified() override;
	void clearActiveDocumentModified() override;
	bool isActiveDocumentModified() const override;
	IPluginLabelingHost* labelingHost() override;
	const IPluginLabelingHost* labelingHost() const override;

private:
	PluginHostContext& m_host;
};

class PluginUiContextImpl : public IPluginUiContext
{
public:
	explicit PluginUiContextImpl(PluginHostContext& host) : m_host(host) {}

	unsigned int contextVersion() const override { return 0x00010000; }

	QDockWidget* registerDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area) override;
	QWidget* sidePanelTabParent() const override;
	int registerSidePanelTab(const char* titleUtf8, QWidget* widget) override;
	void unregisterSidePanelTab(QWidget* widget) override;
	void setSidePanelTabTitle(QWidget* widget, const char* titleUtf8) override;
	QMenu* registerMenuPath(const QStringList& path) override;
	QAction* registerAction(QMenu* menu, const QString& text, std::function<void()> handler) override;
	void setCentralAlternateWidget(QWidget* widget) override;
	void showCentralScene3D() override;
	void showCentralAlternate() override;
	bool isShowingCentralAlternate() const override;
	void enterAlternateSideUi(QWidget* leftPanel, QWidget* rightPanel) override;
	void exitAlternateSideUi() override;
	bool embedActiveRenderWidget(QWidget* slot, QString* outError) override;
	void restoreActiveRenderWidget() override;
	void setModeToolBar(QWidget* toolBar) override;
	void claimWorkspaceMode(const QString& modeId) override;
	void onWorkspaceModeClaimed(std::function<void(const QString& modeId)> callback) override;
	QString currentWorkspaceMode() const override;
	void registerWorkspaceMode(const QString& modeId, const QString& titleZh, const QString& titleEn,
							   std::function<void()> enterFn) override;
	void returnToMainWorkspace() override;
	void enterWorkspaceMode(const QString& modeId) override;

private:
	PluginHostContext& m_host;
};

class PluginGeometryContextImpl : public IPluginGeometryContext
{
public:
	explicit PluginGeometryContextImpl(PluginHostContext& host) : m_host(host) {}

	unsigned int contextVersion() const override { return 0x00010000; }

	bool createPrimitiveMesh(const PluginPrimitiveMeshParams& params, const PluginPrimitiveMeshQuality& quality,
							 const PluginMeshCreateOptions& options, QString* outError,
							 QString* outBackendId) override;
	bool registerBackendType(const PluginBackendMeta& meta, QString* outError) override;
	bool registerTriangleMesh(const std::vector<float>& triangleSoup, const PluginMeshCreateOptions& options,
							  QString* outError) override;
	bool booleanMesh(PluginMeshBooleanOp op, const std::string& targetBackendId, const std::string& toolBackendId,
					 const PluginBooleanMeshOptions& options, std::string* outResultBackendId,
					 QString* outError) override;
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
	IPluginGeometryHost* geometryHost() override;
	const IPluginGeometryHost* geometryHost() const override;
	bool captureActiveViewportPng(QByteArray& outPng, QString* outError) override;
	IPluginPointCloudHost* pointCloudHost() override;
	const IPluginPointCloudHost* pointCloudHost() const override;

private:
	PluginHostContext& m_host;
};

class PluginAiContextImpl : public IPluginAiContext
{
public:
	explicit PluginAiContextImpl(PluginHostContext& host) : m_host(host) {}

	unsigned int contextVersion() const override { return 0x00010000; }

	IAiAssistantHost* aiAssistantHost() override;
	const IAiAssistantHost* aiAssistantHost() const override;
	bool resolveTrajectoryWorkpiece(QString& outBackendId, QString& outStepPath, QString* outError) override;
	bool buildTrajectoryFeatureCatalogSlice(const QString& backendId, const QString& stepPathUtf8,
											const QString& userText, QByteArray& outFullCatalogUtf8,
											QByteArray& outSliceUtf8, QString* outError) override;
	bool showAiFeatureCandidatePreview(const QByteArray& catalogSliceUtf8, QString* outError) override;
	void clearAiFeatureCandidatePreview() override;
	bool commitAiTrajectoryFeatures(const QByteArray& featurePlanJsonUtf8, QString* outSummary,
									QString* outError) override;
	int proposeAndConfirmTrajectoryPlan(const QByteArray& planInUtf8, QByteArray& planOutUtf8, QString* outError,
										bool showRetry) override;
	bool loadBoundTrajectoryPlanForAi(QByteArray& planOutUtf8, QString* outError) override;
	bool reviseAiTrajectoryPlan(const QByteArray& planJsonUtf8, QString* outSummary, QString* outError) override;
	void onParametricBodyHistoryChanged(
		std::function<void(const QString& documentId, const QString& backendId)> callback) override;
	void setProcessFlowAiBridge(IProcessFlowAiBridge* bridge) override;
	IProcessFlowAiBridge* processFlowAiBridge() override;
	const IProcessFlowAiBridge* processFlowAiBridge() const override;

private:
	PluginHostContext& m_host;
};

class PluginRobotContextImpl : public IPluginRobotContext
{
public:
	explicit PluginRobotContextImpl(PluginHostContext& host) : m_host(host) {}

	unsigned int contextVersion() const override { return 0x00010000; }

	IPluginRobotHost* robotHost() override;
	const IPluginRobotHost* robotHost() const override;

private:
	PluginHostContext& m_host;
};

class PluginJobContextImpl : public IPluginJobContext
{
public:
	explicit PluginJobContextImpl(PluginHostContext& host) : m_host(host) {}

	unsigned int contextVersion() const override { return 0x00010000; }

	void invokeOnUiThread(std::function<void()> fn) override;
	void enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
					std::function<void(bool threw, const QString& throwMessage)> onFinished) override;
	quint64 enqueueCancellableJob(const QString& title, PluginCancellableJobWorkFn work,
								  std::function<void(bool threw, const QString& throwMessage)> onFinished) override;
	bool cancelJob(quint64 jobId) override;

private:
	PluginHostContext& m_host;
};

class PluginProjectContextImpl : public IPluginProjectContext
{
public:
	explicit PluginProjectContextImpl(PluginHostContext& host) : m_host(host) {}

	unsigned int contextVersion() const override { return 0x00010000; }

	void onProjectAboutToSave(std::function<void(const QString& documentId, QJsonObject& root)> callback) override;
	void onProjectLoaded(std::function<void(const QString& documentId, const QJsonObject& root)> callback) override;
	std::string importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud,
											 std::string* outError) override;

private:
	PluginHostContext& m_host;
};

#endif // CLOUDSIMHOST_PLUGINCONTEXTIMPLS_H
