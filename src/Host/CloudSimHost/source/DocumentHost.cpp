/// @file DocumentHost.cpp
/// @brief �ĵ������볡���Ž�

#include "DocumentHost.h"

#include "IOsgWidgetView.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BackendFollowReverseIndex.h"
#include "BackendHierarchyModel.h"
#include "BackendSceneDocumentFacade.h"
#include "CloudSimHost.h"
#include "DocumentHostEvents.h"
#include "FollowAttachmentComponent.h"
#include "HeadlessInstructionPropertyDelegate.h"
#include "HeadlessPointCloudBridge.h"
#include "HeadlessRobotContext.h"
#include "HeadlessTrajectorySession.h"
#include "HostRenderViewFactory.h"
#include "IRobotInstructionPropertyDelegate.h"
#include "IRobotSimulationDocument.h"
#include "IRobotUrdfImportContext.h"
#include "MeshBackendData.h"
#include "NullCoreServices.h"
#include "PointCloudBackendData.h"
#include "RobotProgramStore.h"
#include "adapters/DataServiceAdapter.h"
#include "headless/HeadlessAiBridge.h"
#include "headless/HeadlessDrawingBridge.h"
#include "headless/HeadlessGeometryBridge.h"
#include "headless/HeadlessGeomodelBridge.h"
#include "headless/HeadlessLabelingBridge.h"
#include "headless/HeadlessProcessFlowBridge.h"
#include "headless/HeadlessProgramEditBridge.h"
#include "headless/HeadlessRobotCollisionBridge.h"
#include "headless/HeadlessRobotExportBridge.h"
#include "headless/HeadlessRobotPlaybackBridge.h"
#include "io/CustomDeviceHostOps.h"
#include "io/IoSignalNetwork.h"
#include "HostOsgFlavorHooks.h"
#include "adapters/RobotServiceAdapter.h"
#include "visual/BackendVisualSyncEngine.h"

#include <QVBoxLayout>
#include <Qt>

namespace cloudsim::host
{
DocumentHost::DocumentHost(QWidget* parent, cloudsim::core::EventHub& events, const QString& documentId)
	: DocumentHost(parent, events, documentId, true)
{
}

// enableOsgView������ؿ���Web Headless �أ��������ش������� OSG/OpenGL
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

	if (enableOsgView && hostOsgFlavorEnabled())
	{
		FlavorOsgViewportMount mounted = mountFlavorOsgViewport(
			*this, *m_centralLayout, *m_backend,
			[this](const std::string& id, const std::uint32_t aspects)
			{ m_visualSyncEngine.markDirty(id, static_cast<VisualAspect>(aspects), VisualChangeReason::FkWrite); });
		m_osgWidget = mounted.osgWidget;
		m_osgView = mounted.osgView;
		m_osgPane = mounted.osgPane;
		m_renderView = std::move(mounted.renderView);
	}
	else
	{
		m_osgWidget = nullptr;
		m_osgView = nullptr;
		m_osgPane = nullptr;
		m_sceneBridge.setOsgWidget(nullptr);
		// �� Null������ layout�������� NullRenderView �ڲ� unique_ptr ˫���й�
		m_renderView = cloudsim::core::makeNullRenderViewFactory()->createView(this);
		if (auto* nullW = m_renderView->widget())
		{
			nullW->setAttribute(Qt::WA_DontShowOnScreen, true);
			nullW->hide();
		}
		// Web���� DocumentPage���ڴ˹� FK/URDF ��켣�Ự
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
					 [this](const QString& ownerId)
					 {
						 if (m_ioSignalNetwork->ownerKind(ownerId) == IoSignalOwnerKind::Device)
						 {
							 processCustomDevicePoseRisingEdges(*this, *m_ioSignalNetwork, ownerId);
							 return;
						 }
						 // ������ DO ���ʱ��ɨ�豸���� propagate �������豸�¼�����
						 for (const QString& id : m_ioSignalNetwork->ownerIds())
						 {
							 if (m_ioSignalNetwork->ownerKind(id) == IoSignalOwnerKind::Device)
								 processCustomDevicePoseRisingEdges(*this, *m_ioSignalNetwork, id);
						 }
					 });
	QObject::connect(m_ioSignalNetwork.get(), &IoSignalNetwork::networkChanged, this,
					 [this]() { primeCustomDevicePoseEdgeMemory(*m_ioSignalNetwork); });

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
	if (m_osgPane)
	{
		m_osgPane->show();
	}
}

void DocumentHost::showCentralAlternate()
{
	if (!m_centralAlternate || !m_centralLayout)
	{
		return;
	}
	// �� embed ����ģҳʱ�� hide OSG��OSG �� alternate ������
	if (m_osgPane && !m_osgEmbedded)
	{
		m_osgPane->hide();
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
		   (m_osgEmbedded || !m_osgPane || m_osgPane->isHidden());
}

QWidget* DocumentHost::centralAlternateWidget() const
{
	return m_centralAlternate;
}

bool DocumentHost::embedRenderWidget(QWidget* slot, QString* outError)
{
	if (!m_osgPane || !slot)
	{
		if (outError)
			*outError = QStringLiteral("Missing OsgWidget or slot.");
		return false;
	}
	if (m_osgEmbedded && m_osgEmbedSlot == slot)
	{
		m_osgPane->show();
		return true;
	}
	if (m_osgEmbedded)
	{
		restoreRenderWidget();
	}
	if (m_centralLayout)
	{
		m_centralLayout->removeWidget(m_osgPane);
	}
	QVBoxLayout* layout = qobject_cast<QVBoxLayout*>(slot->layout());
	if (!layout)
	{
		layout = new QVBoxLayout(slot);
		layout->setContentsMargins(0, 0, 0, 0);
		layout->setSpacing(0);
	}
	m_osgPane->setParent(slot);
	layout->addWidget(m_osgPane);
	m_osgPane->show();
	m_osgEmbedSlot = slot;
	m_osgEmbedded = true;
	return true;
}

void DocumentHost::restoreRenderWidget()
{
	if (!m_osgEmbedded || !m_osgPane)
	{
		m_osgEmbedded = false;
		m_osgEmbedSlot = nullptr;
		return;
	}
	QWidget* osgPane = m_osgPane;
	if (QLayout* lay = osgPane->parentWidget() ? osgPane->parentWidget()->layout() : nullptr)
	{
		lay->removeWidget(osgPane);
	}
	osgPane->setParent(this);
	if (m_centralLayout)
	{
		m_centralLayout->addWidget(osgPane);
	}
	osgPane->show();
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

void DocumentHost::resetHeadlessGeomodelHistory()
{
	if (m_headlessGeomodelBridge)
		m_headlessGeomodelBridge->resetHistoryStack();
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

const IBackendDataQuery& DocumentHost::dataQuery() const
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

bool DocumentHost::backendContains(const std::string& id) const
{
	return m_backend && m_backend->contains(id);
}

bool DocumentHost::backendRegisterData(const std::shared_ptr<BackendDataBase>& obj)
{
	return m_backend && m_backend->registerData(obj);
}

bool DocumentHost::backendUnregisterData(const std::string& id)
{
	return m_backend && m_backend->unregisterData(id);
}

bool DocumentHost::backendAttachChild(const std::string& parentId, const std::string& childId)
{
	return m_backend && m_backend->attachChild(parentId, childId);
}

bool DocumentHost::backendSetParent(const std::string& childId, const std::string& parentId)
{
	return m_backend && m_backend->setParent(childId, parentId);
}

std::vector<std::string> DocumentHost::backendParentsOf(const std::string& id) const
{
	if (!m_backend)
	{
		return {};
	}
	return m_backend->parentsOf(id);
}

std::vector<std::string> DocumentHost::backendChildrenOf(const std::string& id) const
{
	if (!m_backend)
	{
		return {};
	}
	return m_backend->childrenOf(id);
}

std::vector<std::string> DocumentHost::backendTopoOrder() const
{
	if (!m_backend)
	{
		return {};
	}
	return m_backend->topoOrder();
}

std::vector<std::pair<std::string, std::string>> DocumentHost::backendListEdges() const
{
	if (!m_backend)
	{
		return {};
	}
	return m_backend->listEdges();
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
	return BackendSceneDocumentFacade(data(), backend(), sceneBridge(), followReverseIndex(), osgView());
}

bool DocumentHost::loadMeshFromBackendIntoScene(const MeshBackendData& data, QString* errorMessage,
												const bool resetViewToHome, const bool showWireOutline,
												const bool useSceneLighting)
{
	IOsgWidgetView* osg = osgView();
	if (!osg)
	{
		return false;
	}
	return osg->loadMeshFromBackendData(data, errorMessage, resetViewToHome, showWireOutline, useSceneLighting);
}

bool DocumentHost::loadUrdfLinkMeshIntoScene(const MeshBackendData& data, QString* errorMessage)
{
	IOsgWidgetView* osg = osgView();
	if (!osg)
	{
		return true;
	}
	return osg->loadMeshFromBackendData(data, errorMessage, true, true, true);
}

void DocumentHost::clearStagingGeometry()
{
	if (IOsgWidgetView* osg = osgView())
	{
		osg->clearStagingGeometry();
	}
}

void DocumentHost::syncSceneBackendParent(const std::string& childBackendId, const std::string& parentBackendId)
{
	if (m_osgPane)
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
		if (IOsgWidgetView* osg = osgView())
		{
			osg->removeBackendObjectVisual(id.toStdString());
		}
		publishBackendObjectRemoved(*this, id);
	}
	// ������ clearRobotSimulationIfContains ���룺ɾ������ж�����������ʵ��
	if (m_headlessRobotContext)
	{
		for (const QString& id : ids)
		{
			m_headlessRobotContext->clearRobotSimulationIfContains(id);
		}
	}
	m_followReverseIndex.invalidate(); // ����ɾ���� follower ���˿��ܶ���
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
	// P3-2: ���ݱհ����� followReverseIndex��O(1) ��ѯ�������ÿ��ȫ��ɨ�轨 targetToFollowers
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
		// ֱ�� follower��O(1) ������ѯ��
		for (const std::string& f : followReverseIndex().followersOf(data(), u))
		{
			stack.push_back(f);
		}
		// �㼶�ӽڵ�
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
	// �Զ����豸 Link ������ applyQ FK дλ�ˣ��� URDF ����ͬ����
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
	// �� facade ����֧�����ٵ� ensureVisual�������޷�֧ʱ����ݹ�
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
	// ������ false���� Null ��Ⱦ�������� OsgWidget������ createDocumentHost ���� OSG��
	auto host = std::make_unique<DocumentHost>(nullptr, events, documentId, false);
	host->setAttribute(Qt::WA_DontShowOnScreen, true);
	host->hide();
	host->setOwnedInstructionPropertyDelegate(std::make_unique<HeadlessInstructionPropertyDelegate>(*host));
	return host;
}

std::unique_ptr<core::IRenderViewFactory> createHostRenderViewFactory()
{
	return createFlavorRenderViewFactory();
}

DocumentHost* documentHostFromScope(core::IDocumentScope* scope)
{
	return dynamic_cast<DocumentHost*>(scope);
}

} // namespace cloudsim::host
