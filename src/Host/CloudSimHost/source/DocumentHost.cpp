#include "DocumentHost.h"

#include "adapters/DataServiceAdapter.h"
#include "adapters/OsgRenderViewAdapter.h"
#include "adapters/RobotServiceAdapter.h"

#include "BackendDataManager.h"
#include "BackendFollowReverseIndex.h"
#include "BackendHierarchyModel.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "HostRenderViewFactory.h"
#include "OsgWidgetSceneBridge.h"
#include "RobotProgramStore.h"

#include <QVBoxLayout>

namespace cloudsim::host {

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

	m_dataService = std::make_unique<DataServiceAdapter>(*m_backend);
	m_robotService = std::make_unique<RobotServiceAdapter>(*m_robotProgramStore);
	m_renderView = std::make_unique<OsgRenderViewAdapter>(*m_osgWidget);
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

OsgWidget* DocumentHost::osgWidget() const
{
	return m_osgWidget;
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

BackendFollowReverseIndex& DocumentHost::followReverseIndex()
{
	return m_followReverseIndex;
}

OsgWidgetSceneBridge& DocumentHost::sceneBridge()
{
	return m_sceneBridge;
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
	}
	m_followReverseIndex.invalidate();
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
