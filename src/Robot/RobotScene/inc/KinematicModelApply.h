#ifndef ROBOTSCENE_KINEMATICMODELAPPLY_H
#define ROBOTSCENE_KINEMATICMODELAPPLY_H

/// @file KinematicModelApply.h
/// @brief KinematicModelApply 接口

#include "robot_scene_global.h"

#include "RobotKinematicApplyContext.h"

#include <QVector>
#include <string>
#include <vector>

class BackendDataManager;
class CustomDeviceBackendData;
class IRobotBackendPoseSink;

namespace KinematicModelApply
{
ROBOT_SCENE_API bool applyCustomDevice(const std::string& registryKey, CustomDeviceBackendData& device,
									   BackendDataManager* mgr, IRobotBackendPoseSink* sink,
									   const std::vector<double>& q);

ROBOT_SCENE_API bool applyRobotArm(const std::string& registryKey, const RobotKinematicApplyContext::Context& ctx,
								   const std::vector<double>& localArmQ, QVector<double>& aggregatedAnglesRad);
} // namespace KinematicModelApply

#endif // ROBOTSCENE_KINEMATICMODELAPPLY_H
