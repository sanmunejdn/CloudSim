#pragma once

#include "robot_scene_global.h"

#include <QHash>
#include <QString>
#include <QVector>

#include <osg/Matrixd>

#include <string>
#include <unordered_map>

class IRobotSimulationDocument;
class IRobotBackendPoseSink;

namespace RobotSceneKinematics
{

/// Uses bind pose from the document (T0, outer PAT world matrices) to apply joint angles to the OSG scene.
ROBOT_SCENE_API bool applyJointAnglesFromDocument(
	IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, const QVector<double>& anglesRad);

/// Given per-link mesh world matrices Tq, zero bind FK T0, and bind-time outer PAT matrices, updates the scene.
ROBOT_SCENE_API void applyMeshWorldMatricesRelativeToBind(
	IRobotBackendPoseSink* osg,
	const QHash<QString, osg::Matrixd>& meshWorldByLink,
	const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
	const QHash<QString, QString>& linkNameToBackendId,
	const std::unordered_map<std::string, osg::Matrixd>& outerWorldAtBind);

} // namespace RobotSceneKinematics
