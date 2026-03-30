#include "DocumentPage.h"

#include <QSet>
#include <QTabWidget>
#include <QVBoxLayout>

#include "BackendDataManager.h"
#include "OsgWidget.h"

namespace
{

QStringList collectBackendSubtreeIds(const QString& rootId, const QMap<QString, QString>& parentMap)
{
	QStringList out;
	QStringList queue;
	QSet<QString> queued;
	queue.append(rootId);
	queued.insert(rootId);
	while (!queue.isEmpty())
	{
		const QString id = queue.takeFirst();
		out.append(id);
		for (auto it = parentMap.constBegin(); it != parentMap.constEnd(); ++it)
		{
			if (it.value() == id)
			{
				const QString child = it.key();
				if (!queued.contains(child))
				{
					queued.insert(child);
					queue.append(child);
				}
			}
		}
	}
	return out;
}

} // namespace

QStringList DocumentPage::removeBackendSubtree(const QString& rootBackendId)
{
	if (rootBackendId.isEmpty())
	{
		return {};
	}
	const QStringList ids = collectBackendSubtreeIds(rootBackendId, m_backendParentId);
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

void DocumentPage::setRobotSimulationContext(const QString& importParentBackendId,
	const QString& urdfAbsolutePath,
	const QHash<QString, QString>& linkNameToBackendId,
	const QStringList& revoluteJointNames,
	const QVector<double>& jointLowerRad,
	const QVector<double>& jointUpperRad)
{
	m_robotFkMeshWorldT0.clear();
	m_robotOuterWorldAtBind.clear();
	m_robotImportParentId = importParentBackendId;
	m_robotUrdfAbsolutePath = urdfAbsolutePath;
	m_robotLinkNameToBackendId = linkNameToBackendId;
	m_robotRevoluteJointNames = revoluteJointNames;
	m_robotJointLowerRad = jointLowerRad;
	m_robotJointUpperRad = jointUpperRad;
}

void DocumentPage::setRobotKinematicsBind(const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
	const QHash<QString, osg::Matrixd>& outerWorldByBackendId)
{
	m_robotFkMeshWorldT0 = fkMeshWorldT0;
	m_robotOuterWorldAtBind = outerWorldByBackendId;
}

void DocumentPage::clearRobotSimulationContext()
{
	m_robotImportParentId.clear();
	m_robotUrdfAbsolutePath.clear();
	m_robotLinkNameToBackendId.clear();
	m_robotRevoluteJointNames.clear();
	m_robotJointLowerRad.clear();
	m_robotJointUpperRad.clear();
	m_robotFkMeshWorldT0.clear();
	m_robotOuterWorldAtBind.clear();
}

void DocumentPage::clearRobotSimulationIfContains(const QString& removedBackendId)
{
	if (m_robotImportParentId.isEmpty() && m_robotLinkNameToBackendId.isEmpty())
	{
		return;
	}
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

bool DocumentPage::hasRobotSimulationContext() const
{
	return !m_robotLinkNameToBackendId.isEmpty() && !m_robotUrdfAbsolutePath.isEmpty();
}

bool DocumentPage::hasRobotKinematicsBind() const
{
	return !m_robotFkMeshWorldT0.isEmpty() && !m_robotOuterWorldAtBind.isEmpty();
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

