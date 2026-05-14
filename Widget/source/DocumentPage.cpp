#include "DocumentPage.h"

#include <QSet>
#include <QTabWidget>
#include <QVBoxLayout>

#include "BackendDataManager.h"
#include "BackendDataBase.h"
#include "BackendRelations.h"
#include "FollowAttachmentComponent.h"
#include "OsgWidget.h"

#include <memory>
#include <unordered_map>
#include <vector>

QStringList DocumentPage::removeBackendSubtree(const QString& rootBackendId)
{
	if (rootBackendId.isEmpty())
	{
		return {};
	}
	QStringList ids;
	ids.append(rootBackendId);
	const std::shared_ptr<BackendDataBase> rootObject = m_backend.getData(rootBackendId.toStdString());
	const std::vector<std::string> descendantIds = rootObject
		? backend_relations::descendantIds(*rootObject, m_backend)
		: std::vector<std::string>{};
	for (const std::string& descId : descendantIds)
	{
		ids.append(QString::fromStdString(descId));
	}
	for (const QString& id : ids)
	{
		m_backend.unregisterData(id.toStdString());
		m_backendParentId.remove(id);
		m_backendSourcePath.remove(id);
		m_backendSourceType.remove(id);
	}
	return ids;
}

DocumentPage::DocumentPage(QTabWidget* parentTabs)
	: QWidget(parentTabs)
{
	setContentsMargins(0, 0, 0, 0);
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	layout->setSpacing(0);

	m_osgWidget = new OsgWidget(this);
	layout->addWidget(m_osgWidget);
}

void DocumentPage::rebuildHierarchicalRobotAggregates()
{
	m_robotJointTransforms.clear();
	m_robotRevoluteJointNames.clear();
	m_robotJointLowerRad.clear();
	m_robotJointUpperRad.clear();
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		for (const QString& jn : ri.revoluteJointNamesUnprefixed)
		{
			m_robotRevoluteJointNames.append(ri.jointKeyPrefix + jn);
		}
		m_robotJointLowerRad += ri.jointLowerRad;
		m_robotJointUpperRad += ri.jointUpperRad;
		for (auto it = ri.jointTransformsByPrefixedKey.constBegin(); it != ri.jointTransformsByPrefixedKey.constEnd();
			 ++it)
		{
			m_robotJointTransforms.insert(it.key(), it.value());
		}
	}
	m_robotUrdfAbsolutePath = m_hierarchicalRobots.isEmpty() ? QString() : m_hierarchicalRobots.first().urdfAbsolutePath;
	m_robotSceneBackendId = m_hierarchicalRobots.isEmpty() ? QString() : m_hierarchicalRobots.first().sceneBackendId;
}

void DocumentPage::appendHierarchicalRobotSimulationContext(
	const QString& urdfAbsolutePath,
	const QStringList& revoluteJointNamesUnprefixed,
	const QVector<double>& jointLowerRad,
	const QVector<double>& jointUpperRad,
	const QHash<QString, osg::MatrixTransform*>& jointTransformsPrefixedKeys,
	const QString& robotSceneBackendId,
	const QString& jointPrefixRootOverride)
{
	HierarchicalRobotInstance ri;
	ri.urdfAbsolutePath = urdfAbsolutePath;
	ri.sceneBackendId = robotSceneBackendId;
	ri.jointKeyPrefix = jointPrefixRootOverride.isEmpty()
		? (robotSceneBackendId + QStringLiteral("::"))
		: (jointPrefixRootOverride + QStringLiteral("::"));
	ri.revoluteJointNamesUnprefixed = revoluteJointNamesUnprefixed;
	ri.jointLowerRad = jointLowerRad;
	ri.jointUpperRad = jointUpperRad;
	ri.jointTransformsByPrefixedKey = jointTransformsPrefixedKeys;
	m_hierarchicalRobots.append(std::move(ri));
	rebuildHierarchicalRobotAggregates();
}

void DocumentPage::setRobotPerLinkKinematicsBinding(const QString& importKey,
	const QHash<QString, QString>& linkNameToBackendId,
	const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
	const QHash<QString, osg::Matrixd>& outerWorldAtBindByBackendId,
	bool meshVerticesInLinkFrame)
{
	m_robotImportParentId = importKey;
	m_robotLinkNameToBackendId = linkNameToBackendId;
	m_robotFkMeshWorldT0 = fkMeshWorldT0;
	m_robotOuterWorldAtBind = outerWorldAtBindByBackendId;
	m_robotUrdfMeshVerticesInLinkFrame = meshVerticesInLinkFrame;
}

void DocumentPage::setHierarchicalRobotSimulationContext(
	const QString& urdfAbsolutePath,
	const QStringList& revoluteJointNames,
	const QVector<double>& jointLowerRad,
	const QVector<double>& jointUpperRad,
	const QHash<QString, osg::MatrixTransform*>& jointTransforms,
	const QString& robotBackendId,
	osg::Group* robotAssembly)
{
	(void)robotAssembly;
	clearRobotSimulationContext();
	const QString prefix = robotBackendId + QStringLiteral("::");
	QHash<QString, osg::MatrixTransform*> prefixed;
	prefixed.reserve(jointTransforms.size());
	for (auto it = jointTransforms.constBegin(); it != jointTransforms.constEnd(); ++it)
	{
		prefixed.insert(prefix + it.key(), it.value());
	}
	appendHierarchicalRobotSimulationContext(
		urdfAbsolutePath, revoluteJointNames, jointLowerRad, jointUpperRad, prefixed, robotBackendId, QString());
}

osg::MatrixTransform* DocumentPage::robotJointMatrixTransform(const QString& jointName) const
{
	return m_robotJointTransforms.value(jointName);
}

int DocumentPage::robotKinematicInstanceCount() const
{
	return m_hierarchicalRobots.size();
}

QString DocumentPage::robotUrdfAbsolutePathForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
		? m_hierarchicalRobots[instanceIndex].urdfAbsolutePath
		: QString();
}

int DocumentPage::robotRevoluteJointCountForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
		? m_hierarchicalRobots[instanceIndex].revoluteJointNamesUnprefixed.size()
		: 0;
}

QString DocumentPage::robotJointKeyPrefixForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
		? m_hierarchicalRobots[instanceIndex].jointKeyPrefix
		: QString();
}

void DocumentPage::clearRobotSimulationContext()
{
	// 【中文】清除传统成员
	m_robotImportParentId.clear();
	m_robotLinkNameToBackendId.clear();
	m_robotFkMeshWorldT0.clear();
	m_robotOuterWorldAtBind.clear();
	m_robotUrdfMeshVerticesInLinkFrame = false;

	m_hierarchicalRobots.clear();
	rebuildHierarchicalRobotAggregates();
}

void DocumentPage::clearRobotSimulationIfContains(const QString& removedBackendId)
{
	// 【中文】每连杆后端：任一 link 的 mesh 后端被删则清除整台机器人仿真元数据（与 removeBackendSubtree 行为一致）。
	for (auto it = m_robotLinkNameToBackendId.constBegin(); it != m_robotLinkNameToBackendId.constEnd(); ++it)
	{
		if (it.value() == removedBackendId)
		{
			clearRobotSimulationContext();
			return;
		}
	}

	if (!m_robotImportParentId.isEmpty() && removedBackendId == m_robotImportParentId)
	{
		clearRobotSimulationContext();
		return;
	}

	// 【中文】多机器人：按场景后端 ID 移除对应实例
	for (int i = 0; i < m_hierarchicalRobots.size(); ++i)
	{
		if (m_hierarchicalRobots[i].sceneBackendId == removedBackendId)
		{
			m_hierarchicalRobots.removeAt(i);
			rebuildHierarchicalRobotAggregates();
			return;
		}
	}

	// 【中文】检查新架构的机器人场景 ID（兼容旧单实例字段）
	if (!m_robotSceneBackendId.isEmpty() && removedBackendId == m_robotSceneBackendId)
	{
		clearRobotSimulationContext();
		return;
	}
}

bool DocumentPage::hasRobotSimulationContext() const
{
	return !m_hierarchicalRobots.isEmpty() || !m_robotUrdfAbsolutePath.isEmpty() || !m_robotLinkNameToBackendId.isEmpty();
}

bool DocumentPage::hasRobotKinematicsBind() const
{
	// 【中文】新架构：有关节变换节点即可；传统架构：检查 FK 数据
	return !m_robotJointTransforms.isEmpty() ||
		(!m_robotFkMeshWorldT0.isEmpty() && !m_robotOuterWorldAtBind.isEmpty());
}

// 【中文】每连杆 URDF：link → mesh 后端 id（层级导入时为空）。
const QHash<QString, QString>& DocumentPage::robotLinkNameToBackendId() const
{
	return m_robotLinkNameToBackendId;
}

const QHash<QString, osg::Matrixd>& DocumentPage::robotFkMeshWorldT0() const
{
	return m_robotFkMeshWorldT0;
}

const QHash<QString, osg::Matrixd>& DocumentPage::robotOuterWorldAtBind() const
{
	return m_robotOuterWorldAtBind;
}

bool DocumentPage::robotUrdfMeshVerticesInLinkFrame() const
{
	return m_robotUrdfMeshVerticesInLinkFrame;
}

QString DocumentPage::robotImportParentId() const
{
	return m_robotImportParentId;
}

QStringList DocumentPage::robotLinkBackendIds() const
{
	QStringList out;
	out.reserve(m_robotLinkNameToBackendId.size());
	for (auto it = m_robotLinkNameToBackendId.constBegin(); it != m_robotLinkNameToBackendId.constEnd(); ++it)
	{
		out.append(it.value());
	}
	return out;
}

QString DocumentPage::robotJointPrefixRoot() const
{
	if (m_hierarchicalRobots.isEmpty())
	{
		return QString();
	}
	QString p = m_hierarchicalRobots.first().jointKeyPrefix;
	if (p.endsWith(QStringLiteral("::")))
	{
		p.chop(2);
	}
	return p;
}

bool DocumentPage::takeFollowSolveForced()
{
	const bool v = m_followSolveForced;
	m_followSolveForced = false;
	return v;
}

void DocumentPage::notifyRobotKinematicsAppliedToScene()
{
	if (m_robotLinkNameToBackendId.isEmpty())
	{
		return;
	}
	const QString seed = robotSceneBackendId();
	if (!seed.isEmpty())
	{
		markFollowAttachmentDirtyFromBackendMove(m_backend, seed.toStdString());
	}
}

void DocumentPage::markFollowAttachmentDirtyFromBackendMove(const BackendDataManager& mgr, const std::string& seed)
{
	if (seed.empty())
	{
		return;
	}
	std::unordered_map<std::string, std::vector<std::string>> targetToFollowers;
	for (const auto& d : mgr.listData())
	{
		if (!d)
		{
			continue;
		}
		auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(
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

