/// @file DocumentHost.cpp
/// @brief 文档宿主与场景桥接

#include "DocumentHost.h"
#include "CloudSimHost.h"
#include "io/CustomDeviceHostOps.h"
#include "io/IoSignalNetwork.h"
#include "HeadlessRobotContext.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BackendFollowReverseIndex.h"
#include "BackendHierarchyModel.h"
#include "BackendSceneDocumentFacade.h"
#include "DocumentHostEvents.h"
#include "FollowAttachmentComponent.h"
#include "HostRenderViewFactory.h"
#include "HeadlessInstructionPropertyDelegate.h"
#include "HeadlessRobotContext.h"
#include "HeadlessTrajectorySession.h"
#include "HeadlessPointCloudBridge.h"
#include "headless/HeadlessRobotPlaybackBridge.h"
#include "headless/HeadlessRobotExportBridge.h"
#include "headless/HeadlessGeometryBridge.h"
#include "headless/HeadlessAiBridge.h"
#include "headless/HeadlessRobotCollisionBridge.h"
#include "headless/HeadlessProgramEditBridge.h"
#include "headless/HeadlessProcessFlowBridge.h"
#include "headless/HeadlessDrawingBridge.h"
#include "headless/HeadlessGeomodelBridge.h"
#include "headless/HeadlessLabelingBridge.h"
#include "IRobotInstructionPropertyDelegate.h"
#include "IRobotUrdfImportContext.h"
#include "IRobotSimulationDocument.h"
#include "MeshBackendData.h"
#include "NullCoreServices.h"
#include "OsgWidget.h"
#include "OsgWidgetSceneBridge.h"
#include "PointCloudBackendData.h"
#include "RobotProgramStore.h"
#include "adapters/DataServiceAdapter.h"
#ifndef CLOUDSIM_HOST_HEADLESS_ONLY
#include "adapters/OsgRenderViewAdapter.h"
#endif
#include "adapters/RobotServiceAdapter.h"
#include "visual/BackendVisualSyncEngine.h"

#include <Qt>
#include <QVBoxLayout>

namespace cloudsim::host
{
DocumentHost::DocumentHost(QWidget* parent, cloudsim::core::EventHub& events, const QString& documentId)
	: DocumentHost(parent, events, documentId, true)
{
}

// enableOsgView：桌面必开；Web Headless 关，避免隐藏窗仍拉起 OSG/OpenGL
DocumentHost::DocumentHost(QWidget* parent, cloudsim::core::EventHub& events, const QString& documentId,
						   bool enableOsgView)
	: QWidget(parent), m_documentId(documentId), m_events(events)
{
	setContentsMargins(0, 0, 0, 0);
	m_centralLayout = new QVBoxLayout(this);
	m_centralLayout->setContentsMargins(0, 0, 0, 0);
	m_centralLayout->setSpacing(0);

	m_backend = std::make_unique<BackendDataManager>();
	m_robotProgramStore = std::make_unique<RobotProgramStore>();
	m_hierarchyModel = std::make_unique<BackendHierarchyModel>(*m_backend);

#if !defined(CLOUDSIM_HOST_HEADLESS_ONLY)
	if (enableOsgView)
	{
		// OSG 直挂 layout：勿用 QStackedWidget 包 OpenGL，Windows 上会拖视图卡顿
		m_osgWidget = new OsgWidget(this);
		m_osgWidget->setPoseSyncBackendManager(m_backend.get());
		m_osgWidget->setVisualSyncMarkDirty([this](const std::string& id, const std::uint32_t aspects) {
			m_visualSyncEngine.markDirty(id, static_cast<VisualAspect>(aspects), VisualChangeReason::FkWrite);
		});
		m_sceneBridge.setOsgWidget(m_osgWidget);
		m_centralLayout->addWidget(m_osgWidget);
		m_renderView = std::make_unique<OsgRenderViewAdapter>(*m_osgWidget, *this);
	}
	else
#endif
	{
		m_osgWidget = nullptr;
		m_sceneBridge.setOsgWidget(nullptr);
		// 真 Null：不入 layout，避免与 NullRenderView 内部 unique_ptr 双重托管
		m_renderView = cloudsim::core::makeNullRenderViewFactory()->createView(this);
		if (auto* nullW = m_renderView->widget())
		{
			nullW->setAttribute(Qt::WA_DontShowOnScreen, true);
			nullW->hide();
		}
		// Web：无 DocumentPage，在此挂 FK/URDF 与轨迹会话
		m_headlessRobotContext = std::make_unique<HeadlessRobotContext>(*this);
		m_robotUrdfImportContext = m_headlessRobotContext.get();
		m_headlessTrajectorySession = std::make_unique<HeadlessTrajectorySession>(*this);
		m_headlessPointCloudBridge = std::make_unique<HeadlessPointCloudBridge>(*this);
		m_headlessRobotPlaybackBridge = std::make_unique<HeadlessRobotPlaybackBridge>(*this);
		m_headlessRobotExportBridge = std::make_unique<HeadlessRobotExportBridge>(*this);
		m_headlessGeometryBridge = std::make_unique<HeadlessGeometryBridge>(*this);
		m_headlessAiBridge = std::make_unique<HeadlessAiBridge>(*this);
		m_headlessRobotCollisionBridge = std::make_unique<HeadlessRobotCollisionBridge>(*this);
		m_headlessProgramEditBridge = std::make_unique<HeadlessProgramEditBridge>(*this);
		m_headlessProcessFlowBridge = std::make_unique<HeadlessProcessFlowBridge>(*this);
		m_headlessDrawingBridge = std::make_unique<HeadlessDrawingBridge>(*this);
		m_headlessGeomodelBridge = std::make_unique<HeadlessGeomodelBridge>(*this);
		m_headlessLabelingBridge = std::make_unique<HeadlessLabelingBridge>(*this);
	}

	m_ioSignalNetwork = std::make_unique<IoSignalNetwork>(this);
	QObject::connect(m_ioSignalNetwork.get(), &IoSignalNetwork::ownerIoChanged, this,
					 [this](const QString& ownerId) {
						 if (m_ioSignalNetwork->ownerKind(ownerId) == IoSignalOwnerKind::Device)
						 {
							 processCustomDevicePoseRisingEdges(*this, *m_ioSignalNetwork, ownerId);
							 return;
						 }
						 // 机器人 DO 变更时再扫设备，与 propagate 发出的设备事件互补
						 for (const QString& id : m_ioSignalNetwork->ownerIds())
						 {
							 if (m_ioSignalNetwork->ownerKind(id) == IoSignalOwnerKind::Device)
								 processCustomDevicePoseRisingEdges(*this, *m_ioSignalNetwork, id);
						 }
					 });
	QObject::connect(m_ioSignalNetwork.get(), &IoSignalNetwork::networkChanged, this, [this]() {
		primeCustomDevicePoseEdgeMemory(*m_ioSignalNetwork);
	});

	m_dataService = std::make_unique<DataServiceAdapter>(*this);
	m_robotService = std::make_unique<RobotServiceAdapter>(*this, *m_robotProgramStore);
}

void DocumentHost::setCentralAlternateWidget(QWidget* widget)
{
	if (m_centralAlternate == widget)
	{
		return;
	}
	const bool showingAlt = isShowingCentralAlternate();
	if (m_centralAlternate)
	{
		if (m_centralLayout)
		{
			m_centralLayout->removeWidget(m_centralAlternate);
		}
		m_centralAlternate->hide();
		m_centralAlternate->setParent(nullptr);
		m_centralAlternate = nullptr;
	}
	if (!widget)
	{
		showCentralScene3D();
		return;
	}
	m_centralAlternate = widget;
	if (showingAlt)
	{
		showCentralAlternate();
	}
}

void DocumentHost::showCentralScene3D()
{
	if (m_osgEmbedded)
	{
		return;
	}
	if (m_centralAlternate)
	{
		if (m_centralLayout)
		{
			m_centralLayout->removeWidget(m_centralAlternate);
		}
		m_centralAlternate->hide();
	}
	if (m_osgWidget)
	{
		m_osgWidget->show();
	}
}

void DocumentHost::showCentralAlternate()
{
	if (!m_centralAlternate || !m_centralLayout)
	{
		return;
	}
	// 已 embed 到建模页时勿 hide OSG（OSG 是 alternate 子树）
	if (m_osgWidget && !m_osgEmbedded)
	{
		m_osgWidget->hide();
	}
	if (m_centralLayout->indexOf(m_centralAlternate) < 0)
	{
		m_centralLayout->addWidget(m_centralAlternate);
	}
	m_centralAlternate->show();
}

bool DocumentHost::isShowingCentralAlternate() const
{
	return m_centralAlternate && m_centralAlternate->isVisible() &&
		   (m_osgEmbedded || !m_osgWidget || m_osgWidget->isHidden());
}

QWidget* DocumentHost::centralAlternateWidget() const
{
	return m_centralAlternate;
}

bool DocumentHost::embedRenderWidget(QWidget* slot, QString* outError)
{
	if (!m_osgWidget || !slot)
	{
		if (outError)
			*outError = QStringLiteral("Missing OsgWidget or slot.");
		return false;
	}
	if (m_osgEmbedded && m_osgEmbedSlot == slot)
	{
		m_osgWidget->show();
		return true;
	}
	if (m_osgEmbedded)
	{
		restoreRenderWidget();
	}
	if (m_centralLayout)
	{
		m_centralLayout->removeWidget(m_osgWidget);
	}
	QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(slot->layout());
	if (!layout)
	{
		layout = new QVBoxLayout(slot);
		layout->setContentsMargins(0, 0, 0, 0);
		layout->setSpacing(0);
	}
	m_osgWidget->setParent(slot);
	layout->addWidget(m_osgWidget);
	m_osgWidget->show();
	m_osgEmbedSlot = slot;
	m_osgEmbedded = true;
	return true;
}

void DocumentHost::restoreRenderWidget()
{
	if (!m_osgEmbedded || !m_osgWidget)
	{
		m_osgEmbedded = false;
		m_osgEmbedSlot = nullptr;
		return;
	}
	if (QLayout* lay = m_osgWidget->parentWidget() ? m_osgWidget->parentWidget()->layout() : nullptr)
	{
		lay->removeWidget(m_osgWidget);
	}
	m_osgWidget->setParent(this);
	if (m_centralLayout)
	{
		m_centralLayout->addWidget(m_osgWidget);
	}
	m_osgWidget->show();
	m_osgEmbedded = false;
	m_osgEmbedSlot = nullptr;
}

void DocumentHost::setRobotUrdfImportContext(IRobotUrdfImportContext* context)
{
	m_robotUrdfImportContext = context;
}

IRobotUrdfImportContext* DocumentHost::robotUrdfImportContext() const
{
	return m_robotUrdfImportContext;
}

HeadlessRobotContext* DocumentHost::headlessRobotContext() const
{
	return m_headlessRobotContext.get();
}

HeadlessTrajectorySession* DocumentHost::headlessTrajectorySession() const
{
	return m_headlessTrajectorySession.get();
}

HeadlessPointCloudBridge* DocumentHost::headlessPointCloudBridge() const
{
	return m_headlessPointCloudBridge.get();
}

HeadlessRobotPlaybackBridge* DocumentHost::headlessRobotPlaybackBridge() const
{
	return m_headlessRobotPlaybackBridge.get();
}

HeadlessRobotExportBridge* DocumentHost::headlessRobotExportBridge() const
{
	return m_headlessRobotExportBridge.get();
}

HeadlessGeometryBridge* DocumentHost::headlessGeometryBridge() const
{
	return m_headlessGeometryBridge.get();
}

HeadlessAiBridge* DocumentHost::headlessAiBridge() const
{
	return m_headlessAiBridge.get();
}

HeadlessRobotCollisionBridge* DocumentHost::headlessRobotCollisionBridge() const
{
	return m_headlessRobotCollisionBridge.get();
}

HeadlessProgramEditBridge* DocumentHost::headlessProgramEditBridge() const
{
	return m_headlessProgramEditBridge.get();
}

HeadlessProcessFlowBridge* DocumentHost::headlessProcessFlowBridge() const
{
	return m_headlessProcessFlowBridge.get();
}

HeadlessDrawingBridge* DocumentHost::headlessDrawingBridge() const
{
	return m_headlessDrawingBridge.get();
}

HeadlessGeomodelBridge* DocumentHost::headlessGeomodelBridge() const
{
	return m_headlessGeomodelBridge.get();
}

HeadlessLabelingBridge* DocumentHost::headlessLabelingBridge() const
{
	return m_headlessLabelingBridge.get();
}

void DocumentHost::setInstructionPropertyDelegate(IRobotInstructionPropertyDelegate* delegate)
{
	m_instructionPropertyDelegate = delegate;
}

IRobotInstructionPropertyDelegate* DocumentHost::instructionPropertyDelegate() const
{
	return m_instructionPropertyDelegate;
}

void DocumentHost::setOwnedInstructionPropertyDelegate(std::unique_ptr<IRobotInstructionPropertyDelegate> delegate)
{
	m_ownedInstructionPropertyDelegate = std::move(delegate);
	m_instructionPropertyDelegate = m_ownedInstructionPropertyDelegate.get();
}

void DocumentHost::setPerLinkKinematicsHost(IPerLinkKinematicsHost* host)
{
	m_perLinkKinematicsHost = host;
}

IPerLinkKinematicsHost* DocumentHost::perLinkKinematicsHost() const
{
	return m_perLinkKinematicsHost;
}

void DocumentHost::setPerLinkRobotStateAccessor(IPerLinkRobotStateAccessor* accessor)
{
	m_perLinkRobotStateAccessor = accessor;
}

IPerLinkRobotStateAccessor* DocumentHost::perLinkRobotStateAccessor() const
{
	return m_perLinkRobotStateAccessor;
}

void DocumentHost::noteRobotLocalJointAnglesForSceneRoot(const QString& sceneRootBackendId,
														 const QVector<double>& localJointRad)
{
	if (sceneRootBackendId.isEmpty() || localJointRad.isEmpty())
	{
		return;
	}
	m_robotLocalJointQBySceneRoot.insert(sceneRootBackendId, localJointRad);
}

bool DocumentHost::robotLocalJointAnglesForSceneRoot(const QString& sceneRootBackendId, QVector<double>& outLocal) const
{
	const auto it = m_robotLocalJointQBySceneRoot.constFind(sceneRootBackendId);
	if (it == m_robotLocalJointQBySceneRoot.cend() || it.value().isEmpty())
	{
		return false;
	}
	outLocal = it.value();
	return true;
}

DocumentHost::~DocumentHost() = default;

QString DocumentHost::documentId() const
{
	return m_documentId;
}

cloudsim::core::IDataService& DocumentHost::data()
{
	return *m_dataService;
}

cloudsim::core::IRobotService& DocumentHost::robot()
{
	return *m_robotService;
}

cloudsim::core::IRenderView& DocumentHost::render()
{
	return *m_renderView;
}

cloudsim::core::EventHub& DocumentHost::events()
{
	return m_events;
}

BackendDataManager& DocumentHost::backend()
{
	return *m_backend;
}

const BackendDataManager& DocumentHost::backend() const
{
	return *m_backend;
}

std::shared_ptr<BackendDataBase> DocumentHost::findObject(const std::string& id) const
{
	if (id.empty() || !m_backend)
	{
		return {};
	}
	return m_backend->getData(id);
}

std::vector<std::shared_ptr<BackendDataBase>> DocumentHost::listObjects() const
{
	if (!m_backend)
	{
		return {};
	}
	return m_backend->listData();
}

RobotProgramStore& DocumentHost::robotProgramStore()
{
	return *m_robotProgramStore;
}

IoSignalNetwork& DocumentHost::ioSignalNetwork()
{
	return *m_ioSignalNetwork;
}

const IoSignalNetwork& DocumentHost::ioSignalNetwork() const
{
	return *m_ioSignalNetwork;
}

RobotIo::NamedSignalTable& DocumentHost::namedSignalTable()
{
	return m_ioSignalNetwork->primaryTable();
}

const RobotIo::NamedSignalTable& DocumentHost::namedSignalTable() const
{
	return m_ioSignalNetwork->primaryTable();
}

BackendHierarchyModel& DocumentHost::hierarchyModel()
{
	return *m_hierarchyModel;
}

const BackendHierarchyModel& DocumentHost::hierarchyModel() const
{
	return *m_hierarchyModel;
}

BackendFollowReverseIndex& DocumentHost::followReverseIndex()
{
	return m_followReverseIndex;
}

OsgWidgetSceneBridge& DocumentHost::sceneBridge()
{
	return m_sceneBridge;
}

BackendSceneDocumentFacade DocumentHost::sceneFacade()
{
	return BackendSceneDocumentFacade(data(), backend(), sceneBridge(), followReverseIndex(), m_osgWidget);
}

bool DocumentHost::loadMeshFromBackendIntoScene(const MeshBackendData& data, QString* errorMessage,
												const bool resetViewToHome, const bool showWireOutline,
												const bool useSceneLighting)
{
	if (!m_osgWidget)
	{
		return false;
	}
	return m_osgWidget->loadMeshFromBackendData(data, errorMessage, resetViewToHome, showWireOutline, useSceneLighting);
}

bool DocumentHost::loadUrdfLinkMeshIntoScene(const MeshBackendData& data, QString* errorMessage)
{
	if (!m_osgWidget)
	{
		return true;
	}
	return m_osgWidget->loadMeshFromBackendData(data, errorMessage, true, true, true);
}

void DocumentHost::clearStagingGeometry()
{
	if (m_osgWidget)
	{
		m_osgWidget->clearStagingGeometry();
	}
}

void DocumentHost::syncSceneBackendParent(const std::string& childBackendId, const std::string& parentBackendId)
{
	if (m_osgWidget)
	{
		m_sceneBridge.setBackendParent(childBackendId, parentBackendId);
	}
}

void DocumentHost::focusSceneCameraOnBackend(const std::string& backendId)
{
	if (backendId.empty() || !m_renderView)
	{
		return;
	}
	m_renderView->focusCameraOnBackend(QString::fromStdString(backendId));
}

QStringList DocumentHost::removeBackendSubtree(const QString& rootBackendId)
{
	if (rootBackendId.isEmpty())
	{
		return {};
	}
	QStringList ids;
	const std::string rootStd = rootBackendId.toStdString();
	const std::vector<std::string> subtree = m_hierarchyModel->subtreeIds(rootStd);
	if (subtree.empty() && m_backend->contains(rootStd))
	{
		ids.append(rootBackendId);
	}
	else
	{
		for (const std::string& id : subtree)
		{
			ids.append(QString::fromStdString(id));
		}
	}
	for (const QString& id : ids)
	{
		m_backend->unregisterData(id.toStdString());
		m_projectSidecar.parentId().remove(id);
		m_projectSidecar.sourcePath().remove(id);
		m_projectSidecar.sourceType().remove(id);
		if (m_osgWidget)
		{
			m_osgWidget->removeBackendObjectVisual(id.toStdString());
		}
		publishBackendObjectRemoved(*this, id);
	}
	// 与桌面 clearRobotSimulationIfContains 对齐：删子树后卸掉幽灵机器人实例
	if (m_headlessRobotContext)
	{
		for (const QString& id : ids)
		{
			m_headlessRobotContext->clearRobotSimulationIfContains(id);
		}
	}
	m_followReverseIndex.invalidate(); // 子树删除后 follower 拓扑可能断裂
	return ids;
}

void DocumentHost::setProjectFilePath(const QString& path)
{
	m_projectSidecar.setProjectFilePath(path);
}

const QString& DocumentHost::projectFilePath() const
{
	return m_projectSidecar.projectFilePath();
}

std::unordered_set<std::string>& DocumentHost::followDirtyBackendIds()
{
	return m_followState.dirtyBackendIds();
}

void DocumentHost::markFollowAttachmentDirtyFromBackendMove(const std::string& seed)
{
	if (seed.empty())
	{
		return;
	}
	BackendDataManager& mgr = backend();
	// P3-2: 传递闭包改用 followReverseIndex（O(1) 查询），替代每次全量扫描建 targetToFollowers
	std::vector<std::string> stack;
	stack.push_back(seed);
	std::unordered_set<std::string> visited;
	while (!stack.empty())
	{
		const std::string u = stack.back();
		stack.pop_back();
		if (!visited.insert(u).second)
		{
			continue;
		}
		m_followState.dirtyBackendIds().insert(u);
		// 直接 follower（O(1) 索引查询）
		for (const std::string& f : followReverseIndex().followersOf(data(), u))
		{
			stack.push_back(f);
		}
		// 层级子节点
		for (const std::string& c : mgr.childrenOf(u))
		{
			stack.push_back(c);
		}
	}
}

void DocumentHost::invalidateFollowReverseIndex()
{
	m_followReverseIndex.invalidate();
}

void DocumentHost::clearFollowDirtyBackendIds()
{
	m_followState.clearDirtyBackendIds();
}

void DocumentHost::requestFollowSolveForced()
{
	m_followState.requestSolveForced();
}

bool DocumentHost::takeFollowSolveForced()
{
	return m_followState.takeSolveForced();
}

bool DocumentHost::followSolveForcedPending() const
{
	return m_followState.solveForcedPending();
}

bool DocumentHost::isKinematicsOwnedBackend(const std::string& backendId) const
{
	const QString id = QString::fromStdString(backendId);
	const QString src = m_projectSidecar.sourceType().value(id);
	if (src.compare(QStringLiteral("URDF"), Qt::CaseInsensitive) == 0)
	{
		return true;
	}
	// 自定义设备 Link 几何由 applyQ FK 写位姿，与 URDF 连杆同规则
	return src.compare(QStringLiteral("CustomDeviceLink"), Qt::CaseInsensitive) == 0;
}

void DocumentHost::stripKinematicsOwnedFollowAttachments()
{
	bool removed = false;
	for (const auto& d : m_backend->listData())
	{
		if (!d || !isKinematicsOwnedBackend(d->id()))
		{
			continue;
		}
		if (!d->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
		{
			continue;
		}
		d->removeComponent(FollowAttachmentComponent::typeKeyStatic());
		removed = true;
	}
	if (removed)
	{
		invalidateFollowReverseIndex();
	}
}

void DocumentHost::stripHierarchyDrivenFollowAttachments()
{
	bool removed = false;
	for (const auto& d : m_backend->listData())
	{
		if (!d)
		{
			continue;
		}
		const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!follow || !follow->hierarchyDriven())
		{
			continue;
		}
		d->removeComponent(FollowAttachmentComponent::typeKeyStatic());
		removed = true;
	}
	if (removed)
	{
		invalidateFollowReverseIndex();
	}
}

void DocumentHost::setSuppressRobotFollowDirtyNotify(const bool suppress)
{
	m_followState.setSuppressRobotDirtyNotify(suppress);
}

bool DocumentHost::suppressRobotFollowDirtyNotify() const
{
	return m_followState.suppressRobotDirtyNotify();
}

void DocumentHost::setDeferPropertyPanelVisualFullSync(const bool defer)
{
	m_followState.setDeferPropertyPanelVisualFullSync(defer);
}

bool DocumentHost::deferPropertyPanelVisualFullSync() const
{
	return m_followState.deferPropertyPanelVisualFullSync();
}

BackendVisualSyncEngine& DocumentHost::visualSyncEngine()
{
	return m_visualSyncEngine;
}

const BackendVisualSyncEngine& DocumentHost::visualSyncEngine() const
{
	return m_visualSyncEngine;
}

void DocumentHost::markVisualDirty(const std::string& backendId, const VisualAspect aspects)
{
	m_visualSyncEngine.markDirty(backendId, aspects, VisualChangeReason::Manual);
}

bool DocumentHost::flushVisualSync(const FlushPolicy policy)
{
	return m_visualSyncEngine.flush(policy);
}

void DocumentHost::ensureSelectionVisualForBackend(const std::string& backendId, const bool urdfLinkMesh)
{
	const auto obj = m_backend->getData(backendId);
	if (!obj)
	{
		return;
	}
	// 经 facade 建分支；勿再调 ensureVisual，否则无分支时互相递归
	sceneFacade().ensureSelectionVisualForBackend(*obj, urdfLinkMesh);
}

bool DocumentHost::syncOuterPatFromBackendId(const std::string& backendId)
{
	const auto obj = m_backend->getData(backendId);
	if (!obj)
	{
		return false;
	}
	m_visualSyncEngine.markDirty(backendId, VisualAspect::Transform, VisualChangeReason::Manual);
	return m_visualSyncEngine.flushTransform({backendId});
}

QMap<QString, QString>& DocumentHost::backendSourcePath()
{
	return m_projectSidecar.sourcePath();
}

const QMap<QString, QString>& DocumentHost::backendSourcePath() const
{
	return m_projectSidecar.sourcePath();
}

QMap<QString, QString>& DocumentHost::backendSourceType()
{
	return m_projectSidecar.sourceType();
}

const QMap<QString, QString>& DocumentHost::backendSourceType() const
{
	return m_projectSidecar.sourceType();
}

QMap<QString, QString>& DocumentHost::backendParentId()
{
	return m_projectSidecar.parentId();
}

const QMap<QString, QString>& DocumentHost::backendParentId() const
{
	return m_projectSidecar.parentId();
}

std::unique_ptr<core::IDocumentScope> createDocumentHost(QWidget* parent, core::EventHub& events,
														 const QString& documentId)
{
	return std::make_unique<DocumentHost>(parent, events, documentId);
}

std::unique_ptr<core::IDocumentScope> createHeadlessDocumentHost(core::EventHub& events, const QString& documentId)
{
	// 第三参 false：真 Null 渲染，不构造 OsgWidget（桌面 createDocumentHost 仍走 OSG）
	auto host = std::make_unique<DocumentHost>(nullptr, events, documentId, false);
	host->setAttribute(Qt::WA_DontShowOnScreen, true);
	host->hide();
	host->setOwnedInstructionPropertyDelegate(std::make_unique<HeadlessInstructionPropertyDelegate>(*host));
	return host;
}

std::unique_ptr<core::IRenderViewFactory> createHostRenderViewFactory()
{
#if defined(CLOUDSIM_HOST_HEADLESS_ONLY)
	return core::makeNullRenderViewFactory();
#else
	return std::make_unique<HostRenderViewFactory>();
#endif
}

DocumentHost* documentHostFromScope(core::IDocumentScope* scope)
{
	return dynamic_cast<DocumentHost*>(scope);
}

} // namespace cloudsim::host
