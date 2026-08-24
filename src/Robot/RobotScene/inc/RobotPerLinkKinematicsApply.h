#ifndef ROBOTSCENE_ROBOTPERLINKKINEMATICSAPPLY_H
#define ROBOTSCENE_ROBOTPERLINKKINEMATICSAPPLY_H

/// @file RobotPerLinkKinematicsApply.h
/// @brief KinematicCore FK 写 per-link 场景后端（T0/M0 绑定姿不变，仅 Tq 来自 Core）

#include "RobotPerLinkKinematicsSliceOsg.h"
#include "robot_scene_global.h"

#include <QHash>

#include <osg/Matrixd>

class BackendDataManager;
class IRobotBackendPoseSink;

namespace RobotPerLinkKinematicsApply
{
/// Core FK 的 mesh 世界矩阵写回 slice 中各连杆 backend
ROBOT_SCENE_API bool applyLinkWorldFromCoreFk(IRobotBackendPoseSink* osg, BackendDataManager& mgr,
											  const RobotPerLinkKinematicsSlice& slice,
											  const QHash<QString, osg::Matrixd>& meshWorldTq);
}

#endif // ROBOTSCENE_ROBOTPERLINKKINEMATICSAPPLY_H
