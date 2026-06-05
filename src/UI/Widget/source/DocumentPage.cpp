#include "DocumentPage.h"

#include "BackendSceneDocumentFacade.h"
#include "EventHub.h"
#include "RobotProgramStore.h"

#include <osg/Group>
#include <osg/MatrixTransform>

#include <QSet>
#include <QTabWidget>
#include <QUuid>

#include "IDataService.h"
#include "IRobotBackendPoseSink.h"
#include "BackendDataManager.h"
#include "MeshBackendData.h"
#include "OsgScene.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"

#include <memory>
#include <unordered_map>
#include <vector>

DocumentPage::DocumentPage(QTabWidget* parentTabs, cloudsim::core::EventHub& events)
	: DocumentHost(parentTabs, events,
		  QStringLiteral("doc-%1").arg(QUuid::createUuid().toString(QUuid::WithoutBraces)))
{
	setRobotUrdfImportContext(this);
}

IRobotBackendPoseSink* DocumentPage::urdfImportScenePoseSink()
{
	return sceneFacade().poseSink();
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
		if (backendParentId().value(linkBackendId) == ri.sceneBackendId)
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
	const QString name = const_cast<DocumentPage*>(this)->data().displayName(id);
	return name.isEmpty() ? id : name;
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
	out.robotBasePlacementWorld = ri.basePlacementWorld;
	out.meshVerticesInLinkFrame = ri.meshVerticesInLinkFrame;
	return true;
}

int DocumentPage::robotInstanceIndexForPerLinkBackend(const QString& backendId, bool* outIsSceneRoot) const
{
	if (outIsSceneRoot)
	{
		*outIsSceneRoot = false;
	}
	for (int i = 0; i < m_hierarchicalRobots.size(); ++i)
	{
		const HierarchicalRobotInstance& ri = m_hierarchicalRobots[i];
		if (ri.linkNameToBackendId.isEmpty())
		{
			continue;
		}
		if (ri.sceneBackendId == backendId)
		{
			if (outIsSceneRoot)
			{
				*outIsSceneRoot = true;
			}
			return i;
		}
		if (!ri.perLinkBackends)
		{
			continue;
		}
		for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
		{
			if (it.value() == backendId)
			{
				return i;
			}
		}
	}
	return -1;
}

void DocumentPage::setRobotBasePlacementWorldForInstance(const int instanceIndex, const osg::Matrixd& placementWorld)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return;
	}
	m_hierarchicalRobots[instanceIndex].basePlacementWorld = placementWorld;
	rebuildPerLinkLegacyAggregates();
}

void DocumentPage::updateRobotLinkOuterBindFromWorld(
	const int instanceIndex, const QString& linkBackendId, const osg::Matrixd& world)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size() || linkBackendId.isEmpty())
	{
		return;
	}
	HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends)
	{
		return;
	}
	ri.outerWorldAtBindByBackendId.insert(linkBackendId, world);
	rebuildPerLinkLegacyAggregates();
}

void DocumentPage::clearRobotSimulationContext()
{
	robotProgramStore().clear();
	// 清除传统成员
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

// 每连杆 URDF：link → mesh 后端 id（层级导入时为空）
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

QString DocumentPage::selectionRootBackendId(const QString& backendId) const
{
	if (backendId.isEmpty())
	{
		return backendId;
	}
	bool isSceneRoot = false;
	const int instanceIndex = robotInstanceIndexForPerLinkBackend(backendId, &isSceneRoot);
	if (instanceIndex < 0 || isSceneRoot)
	{
		return backendId;
	}
	return m_hierarchicalRobots[instanceIndex].sceneBackendId;
}

QString DocumentPage::robotGizmoAnchorBackendId(const QString& backendId) const
{
	if (backendId.isEmpty())
	{
		return backendId;
	}
	bool isSceneRoot = false;
	const int instanceIndex = robotInstanceIndexForPerLinkBackend(backendId, &isSceneRoot);
	if (instanceIndex < 0 || !robotUsesPerLinkBackendsForInstance(instanceIndex))
	{
		return backendId;
	}
	const QString anchor = robotFrameWorldReferenceBackendId(instanceIndex);
	return anchor.isEmpty() ? backendId : anchor;
}

bool DocumentPage::applyPerLinkRobotFkFromGizmoAnchor(
	const int instanceIndex,
	const QString& anchorLinkBackendId,
	const QVector<double>& jointAnglesRad)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size() || anchorLinkBackendId.isEmpty())
	{
		return false;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends)
	{
		return false;
	}
	IRobotBackendPoseSink* osg = urdfImportScenePoseSink();
	if (!osg)
	{
		return false;
	}
	RobotPerLinkKinematicsSlice slice;
	if (!robotPerLinkKinematicsForInstance(instanceIndex, slice))
	{
		return false;
	}
	osg::Matrixd anchorWorld;
	if (!osg->getBackendRootWorldMatrix(anchorLinkBackendId.toStdString(), anchorWorld))
	{
		return false;
	}
	osg::Matrixd placement;
	if (!RobotSceneKinematics::computeBasePlacementFromAnchorLinkWorld(
			slice, anchorLinkBackendId, jointAnglesRad, anchorWorld, placement))
	{
		return false;
	}
	setRobotBasePlacementWorldForInstance(instanceIndex, placement);
	slice.robotBasePlacementWorld = placement;
	if (!RobotSceneKinematics::applyPerLinkRobotBasePlacement(osg, backend(), slice, jointAnglesRad, placement))
	{
		return false;
	}
	const QString sceneRootId = ri.sceneBackendId;
	const std::shared_ptr<BackendDataBase> rootData = backend().getData(sceneRootId.toStdString());
	if (rootData && rootData->hasPoseProperty())
	{
		osg::Vec3d trans;
		osg::Quat rot;
		osg::Vec3d scale;
		osg::Quat so;
		placement.decompose(trans, rot, scale, so);
		const osg::Vec3f euler = OsgScene::quatToEulerDeg(rot);
		rootData->setPose(BackendVec3{trans.x(), trans.y(), trans.z()});
		rootData->setRotation(BackendVec3{
			static_cast<double>(euler.x()),
			static_cast<double>(euler.y()),
			static_cast<double>(euler.z())});
	}
	notifyRobotKinematicsAppliedToScene();
	return true;
}

void DocumentPage::reconcilePerLinkOuterBindFromScene(
	const int instanceIndex,
	const QVector<double>& jointAnglesRad)
{
	if (instanceIndex < 0 || instanceIndex >= m_hierarchicalRobots.size())
	{
		return;
	}
	const HierarchicalRobotInstance& ri = m_hierarchicalRobots[instanceIndex];
	if (!ri.perLinkBackends || ri.linkNameToBackendId.isEmpty() || ri.urdfAbsolutePath.isEmpty())
	{
		return;
	}
	IRobotBackendPoseSink* osg = urdfImportScenePoseSink();
	if (!osg)
	{
		return;
	}
	RobotPerLinkKinematicsSlice slice;
	if (!robotPerLinkKinematicsForInstance(instanceIndex, slice))
	{
		return;
	}
	QHash<QString, osg::Matrixd> Tq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldMatrices(
			slice.urdfAbsolutePath, jointAnglesRad, Tq, &fkErr, slice.meshVerticesInLinkFrame))
	{
		return;
	}
	const osg::Matrixd Pinv = osg::Matrixd::inverse(slice.robotBasePlacementWorld);
	for (auto it = ri.linkNameToBackendId.constBegin(); it != ri.linkNameToBackendId.constEnd(); ++it)
	{
		const QString& linkName = it.key();
		const QString& linkBackendId = it.value();
		if (linkName.isEmpty() || linkBackendId.isEmpty())
		{
			continue;
		}
		const auto t0It = slice.fkMeshWorldT0.constFind(linkName);
		const auto tqIt = Tq.constFind(linkName);
		if (t0It == slice.fkMeshWorldT0.constEnd() || tqIt == Tq.constEnd())
		{
			continue;
		}
		osg::Matrixd world;
		if (!osg->getBackendRootWorldMatrix(linkBackendId.toStdString(), world))
		{
			continue;
		}
		const osg::Matrixd m0Bind = world * Pinv * osg::Matrixd::inverse(tqIt.value()) * t0It.value();
		updateRobotLinkOuterBindFromWorld(instanceIndex, linkBackendId, m0Bind);
	}
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

void DocumentPage::notifyRobotKinematicsAppliedToScene()
{
	if (suppressRobotFollowDirtyNotify())
	{
		return;
	}
	for (const HierarchicalRobotInstance& ri : m_hierarchicalRobots)
	{
		if (!ri.sceneBackendId.isEmpty())
		{
			markFollowAttachmentDirtyFromBackendMove(ri.sceneBackendId);
		}
	}
}

void DocumentPage::markFollowAttachmentDirtyFromBackendMove(const QString& seedBackendId)
{
	data().markFollowDirtyFromMove(seedBackendId);
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

