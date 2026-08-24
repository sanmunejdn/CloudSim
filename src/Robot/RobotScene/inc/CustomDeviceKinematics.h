#ifndef ROBOTSCENE_CUSTOMDEVICEKINEMATICS_H
#define ROBOTSCENE_CUSTOMDEVICEKINEMATICS_H

/// @file CustomDeviceKinematics.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 自定义设备 W0*T(q) 合成；转调 RobotExternal 数学

#include "robot_scene_global.h"

#include "CustomDeviceBackendData.h"
#include "CustomDeviceRobotMountComponent.h"
#include "RobotExternalAxes.h"

#include <vector>

class BackendDataManager;
class IRobotBackendPoseSink;

namespace CustomDeviceKinematics
{
ROBOT_SCENE_API RobotExternal::RobotExternalAxisConfig toExternalAxisConfig(const CustomDeviceAxisConfig& in);
ROBOT_SCENE_API RobotExternal::RobotExternalAxisConfigSet toExternalAxisConfigSet(const CustomDeviceAxisConfigSet& in);

/// 挂载启用时 W_eff = T_flange_world * T_flange_device；否则 baseWorldW0
ROBOT_SCENE_API BackendMat4 resolveEffectiveDeviceW0(const CustomDeviceBackendData& device, BackendDataManager* mgr);

/// 仅 Link/Joint 图：syncAxes → 写 q → 树状 FK；无图返回 false
ROBOT_SCENE_API bool applyQ(CustomDeviceBackendData& device, BackendDataManager* mgr, IRobotBackendPoseSink* sink,
							const std::vector<double>* qOverride = nullptr);

/// 将 motionCenterFrame 原点变到父连杆局部，写入 motion.originMm；无 Frame 或失败则 false
ROBOT_SCENE_API bool bakeMotionCenterFrameToOriginMm(CustomDeviceAxisConfig& motion, const double parentWorldCm[16],
													 BackendDataManager* mgr);

/// 读父连杆几何世界矩阵并烘焙旋转中心（组装属性面板，图未提交时兜底）
ROBOT_SCENE_API bool bakeJointMotionOriginFromParentGeometry(CustomDeviceAxisConfig& motion,
															 const std::string& parentGeometryBackendId,
															 BackendDataManager* mgr);

/// 按父连杆 FK 世界矩阵烘焙旋转中心（与 applyQ 同一坐标系）
ROBOT_SCENE_API bool bakeJointMotionOriginFromParentLink(CustomDeviceBackendData& device,
														 CustomDeviceAxisConfig& motion,
														 const std::string& parentLinkId, BackendDataManager* mgr);

/// 对已提交 Link/Joint 图：按 motionCenterFrameBackendId 重算各旋转副 originMm
/// @param qForFk 若非空，用该 q 做父连杆 FK（轴控时应传当前 q，否则世界系 Frame 在上游关节变化后枢轴会偏）
ROBOT_SCENE_API void rebakeRotateJointOriginsFromFrames(CustomDeviceBackendData& device, BackendDataManager* mgr,
														const std::vector<double>* qForFk = nullptr);

/// 世界点 → 设备 W0 局部 mm；失败返回 false
ROBOT_SCENE_API bool worldPointToDeviceLocalMm(const BackendMat4& w0, double worldX, double worldY, double worldZ,
											   double outLocal[3]);
/// 世界方向 → 设备 W0 局部（仅旋转部分），并单位化
ROBOT_SCENE_API bool worldDirectionToDeviceLocal(const BackendMat4& w0, double worldDx, double worldDy, double worldDz,
												 double outLocal[3]);

} // namespace CustomDeviceKinematics

#endif // ROBOTSCENE_CUSTOMDEVICEKINEMATICS_H
