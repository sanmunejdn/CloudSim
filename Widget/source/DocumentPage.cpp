#include "DocumentPage.h"

#include <QSet>
#include <QTabWidget>
#include <QVBoxLayout>

#include "BackendDataManager.h"
#include "BackendRelations.h"
#include "OsgWidget.h"

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
	const QString& robotBackendId)
{
	HierarchicalRobotInstance ri;
	ri.urdfAbsolutePath = urdfAbsolutePath;
	ri.sceneBackendId = robotBackendId;
	ri.jointKeyPrefix = robotBackendId + QStringLiteral("::");
	ri.revoluteJointNamesUnprefixed = revoluteJointNamesUnprefixed;
	ri.jointLowerRad = jointLowerRad;
	ri.jointUpperRad = jointUpperRad;
	ri.jointTransformsByPrefixedKey = jointTransformsPrefixedKeys;
	m_hierarchicalRobots.append(std::move(ri));
	rebuildHierarchicalRobotAggregates();
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
		urdfAbsolutePath, revoluteJointNames, jointLowerRad, jointUpperRad, prefixed, robotBackendId);
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

	m_hierarchicalRobots.clear();
	rebuildHierarchicalRobotAggregates();
}

void DocumentPage::clearRobotSimulationIfContains(const QString& removedBackendId)
{
	// 【中文】检查传统成员
	if (!m_robotImportParentId.isEmpty())
	{
		if (removedBackendId == m_robotImportParentId)
		{
			clearRobotSimulationContext();
			return;
		}
		for (auto it = m_robotLinkNameToBackendId.constBegin(); it != m_robotLinkNameToBackendId.constEnd(); ++it)
		{
			if (it.value() == removedBackendId)
			{
				clearRobotSimulationContext();
				return;
			}
		}
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
	// 【中文】新架构：存在层级机器人实例或 URDF 路径；传统架构：检查 link->backend 映射
	return !m_hierarchicalRobots.isEmpty() || !m_robotUrdfAbsolutePath.isEmpty() ||
		(!m_robotLinkNameToBackendId.isEmpty() && !m_robotImportParentId.isEmpty());
}

bool DocumentPage::hasRobotKinematicsBind() const
{
	// 【中文】新架构：有关节变换节点即可；传统架构：检查 FK 数据
	return !m_robotJointTransforms.isEmpty() ||
		(!m_robotFkMeshWorldT0.isEmpty() && !m_robotOuterWorldAtBind.isEmpty());
}

// 【中文】遗留接口实现（返回空数据，新架构不使用）
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

