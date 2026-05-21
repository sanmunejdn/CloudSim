#pragma once

#include "robot_scene_global.h"

#include "IRobotSimulationDocument.h"

#include <QHash>
#include <QString>
#include <QVector>

#include <osg/Matrixd>

#include <string>
#include <unordered_map>

class IRobotSimulationDocument;
class IRobotBackendPoseSink;
class BackendDataManager;

namespace RobotSceneKinematics
{

/// Uses bind pose from the document (T0, outer PAT world matrices) to apply joint angles to the OSG scene.
ROBOT_SCENE_API bool applyJointAnglesFromDocument(
	IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, const QVector<double>& anglesRad);

/// Updates one robot instance slice inside the aggregated angle vector, then applies FK for all instances.
ROBOT_SCENE_API bool applyJointAnglesForInstance(
	IRobotSimulationDocument* doc,
	IRobotBackendPoseSink* osg,
	int instanceIndex,
	const QVector<double>& localAnglesRad,
	QVector<double>& aggregatedAnglesRad);

/// Per-link URDF: FK → \ref IRobotBackendPoseSink::setBackendRootWorldMatrixFromWorld + \ref MeshBackendData pose/rotation.
/// Link updates run in backend-parent order so nested OSG parents already match FK before children are written.
ROBOT_SCENE_API bool applyJointAnglesViaLinkBackends(
	IRobotSimulationDocument* doc,
	IRobotBackendPoseSink* osg,
	BackendDataManager& mgr,
	const QVector<double>& anglesRad,
	const RobotPerLinkKinematicsSlice& slice);

/// Given per-link mesh world matrices Tq, zero bind FK T0, and bind-time outer PAT matrices, updates the scene.
ROBOT_SCENE_API void applyMeshWorldMatricesRelativeToBind(
	IRobotBackendPoseSink* osg,
	const QHash<QString, osg::Matrixd>& meshWorldByLink,
	const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
	const QHash<QString, QString>& linkNameToBackendId,
	const std::unordered_map<std::string, osg::Matrixd>& outerWorldAtBind);

/// Apply FK for one per-link robot and post-multiply \a basePlacementWorld (scene-root pose) on each link outer matrix.
ROBOT_SCENE_API bool applyPerLinkRobotBasePlacement(
	IRobotBackendPoseSink* osg,
	BackendDataManager& mgr,
	const RobotPerLinkKinematicsSlice& slice,
	const QVector<double>& jointAnglesRad,
	const osg::Matrixd& basePlacementWorld);

} // namespace RobotSceneKinematics
