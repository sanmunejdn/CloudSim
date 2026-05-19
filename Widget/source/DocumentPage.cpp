#include "DocumentPage.h"

#include <QSet>
#include <QTabWidget>
#include <QVBoxLayout>

#include "BackendDataManager.h"
#include "BackendDataBase.h"
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
	const std::string rootStd = rootBackendId.toStdString();
	const std::vector<std::string>& subtree = m_hierarchyModel.subtreeIds(rootStd);
	if (subtree.empty() && m_backend.contains(rootStd))
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
		m_backend.unregisterData(id.toStdString());
		m_backendParentId.remove(id);
		m_backendSourcePath.remove(id);
		m_backendSourceType.remove(id);
	}
	m_followReverseIndex.invalidate();
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
	m_sceneBridge.setOsgWidget(m_osgWidget);
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
	rebuildPerLinkLegacyAggregates();
}

void DocumentPage::rebuildPerLinkLegacyAggregates()
{
	m_robotLinkNameToBackendId.clear();
	m_robotFkMeshWorldT0.clear();
	m_robotOuterWorldAtBind.clear();
	m_robotUrdfMeshVerticesInLinkFrame = false;
	int perLinkCount = 0;
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		if (!ri.perLinkBackends)
		{
			continue;
		}
		++perLinkCount;
		if (perLinkCount == 1)
		{
			m_robotLinkNameToBackendId = ri.linkNameToBackendId;
			m_robotFkMeshWorldT0 = ri.fkMeshWorldT0;
			m_robotOuterWorldAtBind = ri.outerWorldAtBindByBackendId;
			m_robotUrdfMeshVerticesInLinkFrame = ri.meshVerticesInLinkFrame;
			m_robotImportParentId = ri.perLinkImportKey;
		}
	}
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
	QString jointPrefix = importKey;
	if (jointPrefix.endsWith(QStringLiteral("_ctx")))
	{
		jointPrefix.chop(4);
	}
	jointPrefix += QStringLiteral("::");

	HierarchicalRobotInstance* target = nullptr;
	for (int i = m_hierarchicalRobots.size() - 1; i >= 0; --i)
	{
		if (m_hierarchicalRobots[i].jointKeyPrefix == jointPrefix)
		{
			target = &m_hierarchicalRobots[i];
			break;
		}
	}
	if (!target && !m_hierarchicalRobots.isEmpty())
	{
		target = &m_hierarchicalRobots.last();
	}
	if (!target)
	{
		return;
	}
	target->perLinkBackends = true;
	target->perLinkImportKey = importKey;
	target->linkNameToBackendId = linkNameToBackendId;
	target->fkMeshWorldT0 = fkMeshWorldT0;
	target->outerWorldAtBindByBackendId = outerWorldAtBindByBackendId;
	target->meshVerticesInLinkFrame = meshVerticesInLinkFrame;
	rebuildPerLinkLegacyAggregates();
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

QString DocumentPage::robotSceneBackendIdForInstance(const int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
		? m_hierarchicalRobots[instanceIndex].sceneBackendId
		: QString();
}

QString DocumentPage::robotFrameWorldReferenceBackendId(const int instanceIndex) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return QString();
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends)
	{
		return ri.sceneBackendId;
	}
	for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
	{
		const QString& linkBackendId = it.value();
		if (m_backendParentId.value(linkBackendId) == ri.sceneBackendId)
		{
			return linkBackendId;
		}
	}
	return ri.linkNameToBackendId.isEmpty() ? ri.sceneBackendId : ri.linkNameToBackendId.constBegin().value();
}

QString DocumentPage::robotDisplayLabelForInstance(const int instanceIndex) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return QString();
	}
	const QString id = m_hierarchicalRobots[instanceIndex].sceneBackendId;
	if (const auto data = m_backend.getData(id.toStdString()))
	{
		if (!data->name().empty())
		{
			return QString::fromStdString(data->name());
		}
	}
	return id;
}

QStringList DocumentPage::robotRevoluteJointNamesForInstance(const int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
		? m_hierarchicalRobots[instanceIndex].revoluteJointNamesUnprefixed
		: QStringList();
}

void DocumentPage::robotJointLimitsForInstance(
	const int instanceIndex,
	QVector<double>& lowerRad,
	QVector<double>& upperRad) const
{
	lowerRad.clear();
	upperRad.clear();
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	lowerRad = ri.jointLowerRad;
	upperRad = ri.jointUpperRad;
}

int DocumentPage::robotJointOffsetInAggregatedVector(const int instanceIndex) const
{
	int offset = 0;
	for (int i = 0; i < instanceIndex && i < m_hierarchicalRobots.size(); ++i)
	{
		offset += m_hierarchicalRobots[i].revoluteJointNamesUnprefixed.size();
	}
	return offset;
}

int DocumentPage::robotInstanceIndexForSceneBackendId(const QString& sceneBackendId) const
{
	for (int i = 0; i < m_hierarchicalRobots.size(); ++i)
	{
		if (m_hierarchicalRobots[i].sceneBackendId == sceneBackendId)
		{
			return i;
		}
	}
	return -1;
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

bool DocumentPage::robotUsesPerLinkBackendsForInstance(int instanceIndex) const
{
	return instanceIndex >= 0 && instanceIndex < m_hierarchicalRobots.size()
		&& m_hierarchicalRobots[instanceIndex].perLinkBackends;
}

bool DocumentPage::robotPerLinkKinematicsForInstance(int instanceIndex, RobotPerLinkKinematicsSlice& out) const
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return false;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends || ri.linkNameToBackendId.isEmpty())
	{
		return false;
	}
	out.urdfAbsolutePath = ri.urdfAbsolutePath;
	out.sceneRootBackendId = ri.sceneBackendId;
	out.linkNameToBackendId = ri.linkNameToBackendId;
	out.fkMeshWorldT0 = ri.fkMeshWorldT0;
	out.outerWorldAtBindByBackendId = ri.outerWorldAtBindByBackendId;
	out.meshVerticesInLinkFrame = ri.meshVerticesInLinkFrame;
	return true;
}

void DocumentPage::clearRobotSimulationContext()
{
	m_robotProgramStore.clear();
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
	for (int i = 0; i < m_hierarchicalRobots.size(); ++i)
	{
		const HierarchicalRobotInstance& ri = m_hierarchicalRobots[i];
		if (ri.sceneBackendId == removedBackendId)
		{
			m_hierarchicalRobots.removeAt(i);
			rebuildHierarchicalRobotAggregates();
			return;
		}
		if (ri.perLinkBackends)
		{
			for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
			{
				if (it.value() == removedBackendId)
				{
					m_hierarchicalRobots.removeAt(i);
					rebuildHierarchicalRobotAggregates();
					return;
				}
			}
		}
	}

	if (!m_robotSceneBackendId.isEmpty() && removedBackendId == m_robotSceneBackendId)
	{
		clearRobotSimulationContext();
	}
}

bool DocumentPage::hasRobotSimulationContext() const
{
	return !m_hierarchicalRobots.isEmpty() || !m_robotUrdfAbsolutePath.isEmpty() || !m_robotLinkNameToBackendId.isEmpty();
}

bool DocumentPage::hasRobotKinematicsBind() const
{
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		if (ri.perLinkBackends && !ri.fkMeshWorldT0.isEmpty() && !ri.outerWorldAtBindByBackendId.isEmpty())
		{
			return true;
		}
	}
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
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		if (!ri.perLinkBackends)
		{
			continue;
		}
		for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
		{
			out.append(it.value());
		}
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
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		if (!ri.sceneBackendId.isEmpty())
		{
			markFollowAttachmentDirtyFromBackendMove(m_backend, ri.sceneBackendId.toStdString());
		}
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

const RobotCoordinate::RobotCoordinateFrameSet& DocumentPage::robotCoordinateFramesForInstance(
	const int instanceIndex) const
{
	static const RobotCoordinate::RobotCoordinateFrameSet kEmpty{};
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return kEmpty;
	}
	return m_hierarchicalRobots[instanceIndex].coordinateFrames;
}

RobotCoordinate::RobotCoordinateFrameSet& DocumentPage::robotCoordinateFramesForInstance(const int instanceIndex)
{
	static RobotCoordinate::RobotCoordinateFrameSet kEmpty{};
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return kEmpty;
	}
	return m_hierarchicalRobots[instanceIndex].coordinateFrames;
}

const RobotCoordinate::RobotUserFrame* DocumentPage::robotActiveUserFrameForInstance(const int instanceIndex) const
{
	return RobotCoordinate::activeUserFrame(robotCoordinateFramesForInstance(instanceIndex));
}

