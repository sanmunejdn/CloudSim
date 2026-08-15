#ifndef ROBOTSCENE_CUSTOMDEVICEKINEMATICS_H
#define ROBOTSCENE_CUSTOMDEVICEKINEMATICS_H

/// @file CustomDeviceKinematics.h
/// @brief 自定义设备 W0*T(q) 合成；转调 RobotExternal 数学

#include "robot_scene_global.h"

#include "CustomDeviceBackendData.h"
#include "RobotExternalAxes.h"

#include <vector>

class BackendDataManager;
class IRobotBackendPoseSink;

namespace CustomDeviceKinematics
{
ROBOT_SCENE_API RobotExternal::RobotExternalAxisConfig toExternalAxisConfig(const CustomDeviceAxisConfig& in);
ROBOT_SCENE_API RobotExternal::RobotExternalAxisConfigSet toExternalAxisConfigSet(const CustomDeviceAxisConfigSet& in);

/// 仅 Link/Joint 图：syncAxes → 写 q → 树状 FK；无图返回 false
ROBOT_SCENE_API bool applyQ(CustomDeviceBackendData& device, BackendDataManager* mgr, IRobotBackendPoseSink* sink,
							const std::vector<double>* qOverride = nullptr);

/// 将 motionCenterFrame 原点变到父连杆局部，写入 motion.originMm；无 Frame 或失败则 false
ROBOT_SCENE_API bool bakeMotionCenterFrameToOriginMm(CustomDeviceAxisConfig& motion, const double parentWorldCm[16],
													 BackendDataManager* mgr);

/// 世界点 → 设备 W0 局部 mm；失败返回 false
ROBOT_SCENE_API bool worldPointToDeviceLocalMm(const BackendMat4& w0, double worldX, double worldY, double worldZ,
											   double outLocal[3]);
/// 世界方向 → 设备 W0 局部（仅旋转部分），并单位化
ROBOT_SCENE_API bool worldDirectionToDeviceLocal(const BackendMat4& w0, double worldDx, double worldDy, double worldDz,
												 double outLocal[3]);

} // namespace CustomDeviceKinematics

#endif // ROBOTSCENE_CUSTOMDEVICEKINEMATICS_H
