#include "DocumentHost.h"

#include "adapters/DataServiceAdapter.h"
#include "adapters/OsgRenderViewAdapter.h"
#include "adapters/RobotServiceAdapter.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFollowReverseIndex.h"
#include "FollowAttachmentComponent.h"
#include "BackendHierarchyModel.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "OsgWidget.h"
#include "HostRenderViewFactory.h"
#include "OsgWidgetSceneBridge.h"
#include "BackendFileImport.h"
#include "DocumentHostEvents.h"
#include "BackendSceneDocumentFacade.h"
#include "IRobotInstructionPropertyDelegate.h"
#include "IRobotUrdfImportContext.h"
#include "RobotProgramStore.h"

#include <QVBoxLayout>

namespace cloudsim::host {

// Data→OSG→三适配器
DocumentHost::DocumentHost(QWidget* parent, cloudsim::core::EventHub& events, const QString& documentId)
	: QWidget(parent)
	, m_documentId(documentId)
	, m_events(events)
{
	setContentsMargins(0, 0, 0, 0);
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(0);

	m_backend = std::make_unique<BackendDataManager>();
	m_robotProgramStore = std::make_unique<RobotProgramStore>();
	m_hierarchyModel = std::make_unique<BackendHierarchyModel>(*m_backend);

	m_osgWidget = new OsgWidget(this);
	m_sceneBridge.setOsgWidget(m_osgWidget);
	layout->addWidget(m_osgWidget);

	m_dataService = std::make_unique<DataServiceAdapter>(*this);
	m_robotService = std::make_unique<RobotServiceAdapter>(*this, *m_robotProgramStore);
	m_renderView = std::make_unique<OsgRenderViewAdapter>(*m_osgWidget, *this);
}

void DocumentHost::setRobotUrdfImportContext(IRobotUrdfImportContext* context)
{
	m_robotUrdfImportContext = context;
}

IRobotUrdfImportContext* DocumentHost::robotUrdfImportContext() const
{
	return m_robotUrdfImportContext;
}

void DocumentHost::setInstructionPropertyDelegate(IRobotInstructionPropertyDelegate* delegate)
{
	m_instructionPropertyDelegate = delegate;
}

IRobotInstructionPropertyDelegate* DocumentHost::instructionPropertyDelegate() const
{
	return m_instructionPropertyDelegate;
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

RobotProgramStore& DocumentHost::robotProgramStore()
{
	return *m_robotProgramStore;
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
	return BackendSceneDocumentFacade(backend(), sceneBridge(), followReverseIndex(), m_osgWidget);
}

bool DocumentHost::loadMeshFromBackendIntoScene(const MeshBackendData& data, QString* errorMessage,
	const bool resetViewToHome, const bool showWireOutline, const bool useSceneLighting)
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
	const std::vector<std::string>& subtree = m_hierarchyModel->subtreeIds(rootStd);
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
		m_backendParentId.remove(id);
		m_backendSourcePath.remove(id);
		m_backendSourceType.remove(id);
		if (m_osgWidget)
		{
			m_osgWidget->removeBackendObjectVisual(id.toStdString());
		}
		publishBackendObjectRemoved(*this, id);
	}
	m_followReverseIndex.invalidate(); // 子树删除后 follower 拓扑可能断裂
	return ids;
}

void DocumentHost::setProjectFilePath(const QString& path)
{
	m_projectFilePath = path;
}

const QString& DocumentHost::projectFilePath() const
{
	return m_projectFilePath;
}

std::unordered_set<std::string>& DocumentHost::followDirtyBackendIds()
{
	return m_followDirtyBackendIds;
}

void DocumentHost::markFollowAttachmentDirtyFromBackendMove(const std::string& seed)
{
	if (seed.empty())
	{
		return;
	}
	BackendDataManager& mgr = backend();
	// follower 链传播脏标记
	std::unordered_map<std::string, std::vector<std::string>> targetToFollowers;
	for (const auto& d : mgr.listData())
	{
		if (!d)
		{
			continue;
		}
		const auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!comp || !comp->enabled())
		{
			continue;
		}
		const std::string tid = comp->targetBackendId();
		if (tid.empty() || tid == d->id())
		{
			continue;
		}
		if (!mgr.contains(tid))
		{
			continue;
		}
		targetToFollowers[tid].push_back(d->id());
	}

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
		m_followDirtyBackendIds.insert(u);
		const auto itF = targetToFollowers.find(u);
		if (itF != targetToFollowers.end())
		{
			for (const std::string& f : itF->second)
			{
				stack.push_back(f);
			}
		}
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
	m_followDirtyBackendIds.clear();
}

void DocumentHost::requestFollowSolveForced()
{
	m_followSolveForced = true;
}

bool DocumentHost::takeFollowSolveForced()
{
	const bool v = m_followSolveForced;
	m_followSolveForced = false;
	return v;
}

bool DocumentHost::followSolveForcedPending() const
{
	return m_followSolveForced;
}

void DocumentHost::setSuppressRobotFollowDirtyNotify(const bool suppress)
{
	m_suppressRobotFollowDirtyNotify = suppress;
}

bool DocumentHost::suppressRobotFollowDirtyNotify() const
{
	return m_suppressRobotFollowDirtyNotify;
}

void DocumentHost::ensureSelectionVisualForBackend(const std::string& backendId, const bool urdfLinkMesh)
{
	const auto obj = m_backend->getData(backendId);
	if (!obj || !m_osgWidget)
	{
		return;
	}
	BackendSceneDocumentFacade facade(*m_backend, m_sceneBridge, m_followReverseIndex, m_osgWidget);
	facade.ensureSelectionVisualForBackend(*obj, urdfLinkMesh);
}

bool DocumentHost::syncOuterPatFromBackendId(const std::string& backendId)
{
	const auto obj = m_backend->getData(backendId);
	if (!obj)
	{
		return false;
	}
	BackendSceneDocumentFacade facade(*m_backend, m_sceneBridge, m_followReverseIndex, m_osgWidget);
	facade.entity(backendId).syncOuterPatFromBackend(*obj);
	return true;
}

QMap<QString, QString>& DocumentHost::backendSourcePath()
{
	return m_backendSourcePath;
}

const QMap<QString, QString>& DocumentHost::backendSourcePath() const
{
	return m_backendSourcePath;
}

QMap<QString, QString>& DocumentHost::backendSourceType()
{
	return m_backendSourceType;
}

const QMap<QString, QString>& DocumentHost::backendSourceType() const
{
	return m_backendSourceType;
}

QMap<QString, QString>& DocumentHost::backendParentId()
{
	return m_backendParentId;
}

const QMap<QString, QString>& DocumentHost::backendParentId() const
{
	return m_backendParentId;
}

std::unique_ptr<core::IDocumentScope> createDocumentHost(QWidget* parent, core::EventHub& events,
	const QString& documentId)
{
	return std::make_unique<DocumentHost>(parent, events, documentId);
}

std::unique_ptr<core::IRenderViewFactory> createHostRenderViewFactory()
{
	return std::make_unique<HostRenderViewFactory>();
}

DocumentHost* documentHostFromScope(core::IDocumentScope* scope)
{
	return dynamic_cast<DocumentHost*>(scope);
}

} // namespace cloudsim::host
