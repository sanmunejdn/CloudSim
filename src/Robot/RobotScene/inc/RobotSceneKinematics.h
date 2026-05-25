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

/// 用文档绑定姿 T0/外支 PAT 将关节角应用到 OSG 场景
ROBOT_SCENE_API bool applyJointAnglesFromDocument(
	IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, const QVector<double>& anglesRad);

/// 更新聚合角向量中某一实例，再 FK 全部实例
ROBOT_SCENE_API bool applyJointAnglesForInstance(
	IRobotSimulationDocument* doc,
	IRobotBackendPoseSink* osg,
	int instanceIndex,
	const QVector<double>& localAnglesRad,
	QVector<double>& aggregatedAnglesRad);

/// per-link URDF：FK → setBackendRootWorldMatrix + MeshBackendData pose；按后端父序写子连杆
ROBOT_SCENE_API bool applyJointAnglesViaLinkBackends(
	IRobotSimulationDocument* doc,
	IRobotBackendPoseSink* osg,
	BackendDataManager& mgr,
	const QVector<double>& anglesRad,
	const RobotPerLinkKinematicsSlice& slice);

/// Tq、零位 FK T0、绑定外 PAT → 更新场景
ROBOT_SCENE_API void applyMeshWorldMatricesRelativeToBind(
	IRobotBackendPoseSink* osg,
	const QHash<QString, osg::Matrixd>& meshWorldByLink,
	const QHash<QString, osg::Matrixd>& fkMeshWorldT0,
	const QHash<QString, QString>& linkNameToBackendId,
	const std::unordered_map<std::string, osg::Matrixd>& outerWorldAtBind);

/// per-link FK 后左乘 basePlacementWorld（场景根位姿）
ROBOT_SCENE_API bool applyPerLinkRobotBasePlacement(
	IRobotBackendPoseSink* osg,
	BackendDataManager& mgr,
	const RobotPerLinkKinematicsSlice& slice,
	const QVector<double>& jointAnglesRad,
	const osg::Matrixd& basePlacementWorld);

} // namespace RobotSceneKinematics
