/// @file PluginContextImpls.cpp
/// @brief 窄接口宿主实现：纯转发到 PluginHostContext，不含业务逻辑

#include "PluginContextImpls.h"

#include "PluginHostContext.h"

#include <utility>

int PluginDocumentContextImpl::documentCount() const
{
	return std::as_const(m_host).documentCount();
}

IPluginDocument* PluginDocumentContextImpl::activeDocument()
{
	return m_host.activeDocument();
}

const IPluginDocument* PluginDocumentContextImpl::activeDocument() const
{
	return std::as_const(m_host).activeDocument();
}

IPluginDocument* PluginDocumentContextImpl::documentAt(int index)
{
	return m_host.documentAt(index);
}

const IPluginDocument* PluginDocumentContextImpl::documentAt(int index) const
{
	return std::as_const(m_host).documentAt(index);
}

IPluginDocument* PluginDocumentContextImpl::documentById(const QString& documentId)
{
	return m_host.documentById(documentId);
}

const IPluginDocument* PluginDocumentContextImpl::documentById(const QString& documentId) const
{
	return std::as_const(m_host).documentById(documentId);
}

void PluginDocumentContextImpl::onActiveDocumentChanged(std::function<void(IPluginDocument*)> callback)
{
	m_host.onActiveDocumentChanged(std::move(callback));
}

void PluginDocumentContextImpl::onDocumentClosed(std::function<void(const QString& documentId)> callback)
{
	m_host.onDocumentClosed(std::move(callback));
}

void PluginDocumentContextImpl::markActiveDocumentModified()
{
	m_host.markActiveDocumentModified();
}

void PluginDocumentContextImpl::clearActiveDocumentModified()
{
	m_host.clearActiveDocumentModified();
}

bool PluginDocumentContextImpl::isActiveDocumentModified() const
{
	return std::as_const(m_host).isActiveDocumentModified();
}

IPluginLabelingHost* PluginDocumentContextImpl::labelingHost()
{
	return m_host.labelingHost();
}

const IPluginLabelingHost* PluginDocumentContextImpl::labelingHost() const
{
	return std::as_const(m_host).labelingHost();
}

QDockWidget* PluginUiContextImpl::registerDockWidget(const QString& title, QWidget* widget, Qt::DockWidgetArea area)
{
	return m_host.registerDockWidget(title, widget, area);
}

QWidget* PluginUiContextImpl::sidePanelTabParent() const
{
	return std::as_const(m_host).sidePanelTabParent();
}

int PluginUiContextImpl::registerSidePanelTab(const char* titleUtf8, QWidget* widget)
{
	return m_host.registerSidePanelTab(titleUtf8, widget);
}

void PluginUiContextImpl::unregisterSidePanelTab(QWidget* widget)
{
	m_host.unregisterSidePanelTab(widget);
}

void PluginUiContextImpl::setSidePanelTabTitle(QWidget* widget, const char* titleUtf8)
{
	m_host.setSidePanelTabTitle(widget, titleUtf8);
}

QMenu* PluginUiContextImpl::registerMenuPath(const QStringList& path)
{
	return m_host.registerMenuPath(path);
}

QAction* PluginUiContextImpl::registerAction(QMenu* menu, const QString& text, std::function<void()> handler)
{
	return m_host.registerAction(menu, text, std::move(handler));
}

void PluginUiContextImpl::setCentralAlternateWidget(QWidget* widget)
{
	m_host.setCentralAlternateWidget(widget);
}

void PluginUiContextImpl::showCentralScene3D()
{
	m_host.showCentralScene3D();
}

void PluginUiContextImpl::showCentralAlternate()
{
	m_host.showCentralAlternate();
}

bool PluginUiContextImpl::isShowingCentralAlternate() const
{
	return std::as_const(m_host).isShowingCentralAlternate();
}

void PluginUiContextImpl::enterAlternateSideUi(QWidget* leftPanel, QWidget* rightPanel)
{
	m_host.enterAlternateSideUi(leftPanel, rightPanel);
}

void PluginUiContextImpl::exitAlternateSideUi()
{
	m_host.exitAlternateSideUi();
}

bool PluginUiContextImpl::embedActiveRenderWidget(QWidget* slot, QString* outError)
{
	return m_host.embedActiveRenderWidget(slot, outError);
}

void PluginUiContextImpl::restoreActiveRenderWidget()
{
	m_host.restoreActiveRenderWidget();
}

void PluginUiContextImpl::setModeToolBar(QWidget* toolBar)
{
	m_host.setModeToolBar(toolBar);
}

void PluginUiContextImpl::claimWorkspaceMode(const QString& modeId)
{
	m_host.claimWorkspaceMode(modeId);
}

void PluginUiContextImpl::onWorkspaceModeClaimed(std::function<void(const QString& modeId)> callback)
{
	m_host.onWorkspaceModeClaimed(std::move(callback));
}

QString PluginUiContextImpl::currentWorkspaceMode() const
{
	return std::as_const(m_host).currentWorkspaceMode();
}

void PluginUiContextImpl::registerWorkspaceMode(const QString& modeId, const QString& titleZh, const QString& titleEn,
												std::function<void()> enterFn)
{
	m_host.registerWorkspaceMode(modeId, titleZh, titleEn, std::move(enterFn));
}

void PluginUiContextImpl::returnToMainWorkspace()
{
	m_host.returnToMainWorkspace();
}

void PluginUiContextImpl::enterWorkspaceMode(const QString& modeId)
{
	m_host.enterWorkspaceMode(modeId);
}

bool PluginGeometryContextImpl::createPrimitiveMesh(const PluginPrimitiveMeshParams& params,
													const PluginPrimitiveMeshQuality& quality,
													const PluginMeshCreateOptions& options, QString* outError,
													QString* outBackendId)
{
	return m_host.createPrimitiveMesh(params, quality, options, outError, outBackendId);
}

bool PluginGeometryContextImpl::registerBackendType(const PluginBackendMeta& meta, QString* outError)
{
	return m_host.registerBackendType(meta, outError);
}

bool PluginGeometryContextImpl::registerTriangleMesh(const std::vector<float>& triangleSoup,
													 const PluginMeshCreateOptions& options, QString* outError)
{
	return m_host.registerTriangleMesh(triangleSoup, options, outError);
}

bool PluginGeometryContextImpl::booleanMesh(PluginMeshBooleanOp op, const std::string& targetBackendId,
											const std::string& toolBackendId, const PluginBooleanMeshOptions& options,
											std::string* outResultBackendId, QString* outError)
{
	return m_host.booleanMesh(op, targetBackendId, toolBackendId, options, outResultBackendId, outError);
}

bool PluginGeometryContextImpl::buildPrimitiveMeshSoup(const PluginPrimitiveMeshParams& params,
													   const PluginPrimitiveMeshQuality& quality,
													   const PluginMeshCreateOptions& placement,
													   std::vector<float>& outWorldSoup, QString* outError)
{
	return m_host.buildPrimitiveMeshSoup(params, quality, placement, outWorldSoup, outError);
}

bool PluginGeometryContextImpl::booleanMeshSoups(PluginMeshBooleanOp op, const std::vector<float>& targetWorldSoup,
												 const std::vector<float>& toolWorldSoup,
												 const PluginBooleanMeshOptions& options,
												 std::string* outResultBackendId, QString* outError)
{
	return m_host.booleanMeshSoups(op, targetWorldSoup, toolWorldSoup, options, outResultBackendId, outError);
}

bool PluginGeometryContextImpl::booleanPrimitiveMeshes(PluginMeshBooleanOp op,
													   const PluginPrimitiveMeshParams& targetParams,
													   const PluginPrimitiveMeshQuality& targetQuality,
													   const PluginMeshCreateOptions& targetPlacement,
													   const PluginPrimitiveMeshParams& toolParams,
													   const PluginPrimitiveMeshQuality& toolQuality,
													   const PluginMeshCreateOptions& toolPlacement,
													   const PluginBooleanMeshOptions& options,
													   std::string* outResultBackendId, QString* outError)
{
	return m_host.booleanPrimitiveMeshes(op, targetParams, targetQuality, targetPlacement, toolParams, toolQuality,
										 toolPlacement, options, outResultBackendId, outError);
}

IPluginGeometryHost* PluginGeometryContextImpl::geometryHost()
{
	return m_host.geometryHost();
}

const IPluginGeometryHost* PluginGeometryContextImpl::geometryHost() const
{
	return std::as_const(m_host).geometryHost();
}

bool PluginGeometryContextImpl::captureActiveViewportPng(QByteArray& outPng, QString* outError)
{
	return m_host.captureActiveViewportPng(outPng, outError);
}

IPluginPointCloudHost* PluginGeometryContextImpl::pointCloudHost()
{
	return m_host.pointCloudHost();
}

const IPluginPointCloudHost* PluginGeometryContextImpl::pointCloudHost() const
{
	return std::as_const(m_host).pointCloudHost();
}

IAiAssistantHost* PluginAiContextImpl::aiAssistantHost()
{
	return m_host.aiAssistantHost();
}

const IAiAssistantHost* PluginAiContextImpl::aiAssistantHost() const
{
	return std::as_const(m_host).aiAssistantHost();
}

bool PluginAiContextImpl::resolveTrajectoryWorkpiece(QString& outBackendId, QString& outStepPath, QString* outError)
{
	return m_host.resolveTrajectoryWorkpiece(outBackendId, outStepPath, outError);
}

bool PluginAiContextImpl::buildTrajectoryFeatureCatalogSlice(const QString& backendId, const QString& stepPathUtf8,
															 const QString& userText, QByteArray& outFullCatalogUtf8,
															 QByteArray& outSliceUtf8, QString* outError)
{
	return m_host.buildTrajectoryFeatureCatalogSlice(backendId, stepPathUtf8, userText, outFullCatalogUtf8,
													 outSliceUtf8, outError);
}

bool PluginAiContextImpl::showAiFeatureCandidatePreview(const QByteArray& catalogSliceUtf8, QString* outError)
{
	return m_host.showAiFeatureCandidatePreview(catalogSliceUtf8, outError);
}

void PluginAiContextImpl::clearAiFeatureCandidatePreview()
{
	m_host.clearAiFeatureCandidatePreview();
}

bool PluginAiContextImpl::commitAiTrajectoryFeatures(const QByteArray& featurePlanJsonUtf8, QString* outSummary,
													 QString* outError)
{
	return m_host.commitAiTrajectoryFeatures(featurePlanJsonUtf8, outSummary, outError);
}

int PluginAiContextImpl::proposeAndConfirmTrajectoryPlan(const QByteArray& planInUtf8, QByteArray& planOutUtf8,
														 QString* outError, bool showRetry)
{
	return m_host.proposeAndConfirmTrajectoryPlan(planInUtf8, planOutUtf8, outError, showRetry);
}

bool PluginAiContextImpl::loadBoundTrajectoryPlanForAi(QByteArray& planOutUtf8, QString* outError)
{
	return m_host.loadBoundTrajectoryPlanForAi(planOutUtf8, outError);
}

bool PluginAiContextImpl::reviseAiTrajectoryPlan(const QByteArray& planJsonUtf8, QString* outSummary,
												 QString* outError)
{
	return m_host.reviseAiTrajectoryPlan(planJsonUtf8, outSummary, outError);
}

void PluginAiContextImpl::onParametricBodyHistoryChanged(
	std::function<void(const QString& documentId, const QString& backendId)> callback)
{
	m_host.onParametricBodyHistoryChanged(std::move(callback));
}

void PluginAiContextImpl::setProcessFlowAiBridge(IProcessFlowAiBridge* bridge)
{
	m_host.setProcessFlowAiBridge(bridge);
}

IProcessFlowAiBridge* PluginAiContextImpl::processFlowAiBridge()
{
	return m_host.processFlowAiBridge();
}

const IProcessFlowAiBridge* PluginAiContextImpl::processFlowAiBridge() const
{
	return std::as_const(m_host).processFlowAiBridge();
}

IPluginRobotHost* PluginRobotContextImpl::robotHost()
{
	return m_host.robotHost();
}

const IPluginRobotHost* PluginRobotContextImpl::robotHost() const
{
	return std::as_const(m_host).robotHost();
}

void PluginJobContextImpl::invokeOnUiThread(std::function<void()> fn)
{
	m_host.invokeOnUiThread(std::move(fn));
}

void PluginJobContextImpl::enqueueJob(const QString& title, std::function<void(const PluginJobProgressFn&)> work,
									  std::function<void(bool threw, const QString& throwMessage)> onFinished)
{
	m_host.enqueueJob(title, std::move(work), std::move(onFinished));
}

quint64 PluginJobContextImpl::enqueueCancellableJob(
	const QString& title, PluginCancellableJobWorkFn work,
	std::function<void(bool threw, const QString& throwMessage)> onFinished)
{
	return m_host.enqueueCancellableJob(title, std::move(work), std::move(onFinished));
}

bool PluginJobContextImpl::cancelJob(quint64 jobId)
{
	return m_host.cancelJob(jobId);
}

void PluginProjectContextImpl::onProjectAboutToSave(
	std::function<void(const QString& documentId, QJsonObject& root)> callback)
{
	m_host.onProjectAboutToSave(std::move(callback));
}

void PluginProjectContextImpl::onProjectLoaded(
	std::function<void(const QString& documentId, const QJsonObject& root)> callback)
{
	m_host.onProjectLoaded(std::move(callback));
}

std::string PluginProjectContextImpl::importFileIntoActiveDocument(const std::string& pathUtf8, bool isPointCloud,
																   std::string* outError)
{
	return m_host.importFileIntoActiveDocument(pathUtf8, isPointCloud, outError);
}
