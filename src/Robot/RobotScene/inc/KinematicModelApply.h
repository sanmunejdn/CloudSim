#ifndef ROBOTSCENE_KINEMATICMODELAPPLY_H
#define ROBOTSCENE_KINEMATICMODELAPPLY_H

#include "RobotKinematicApplyContext.h"
#include "robot_scene_global.h"

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
}

#endif // ROBOTSCENE_KINEMATICMODELAPPLY_H
