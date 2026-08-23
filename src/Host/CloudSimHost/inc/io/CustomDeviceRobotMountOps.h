#ifndef CLOUDSIMHOST_CUSTOMDEVICEROBOTMOUNTOPS_H
#define CLOUDSIMHOST_CUSTOMDEVICEROBOTMOUNTOPS_H

/// @file CustomDeviceRobotMountOps.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 自定义设备挂机器人法兰：Follow + 位姿属性

#include "cloudsim_host_global.h"

#include "BackendFollowMath.h"

#include <QString>
#include <QVector>

class CustomDeviceBackendData;

namespace cloudsim::host
{
class DocumentHost;

/// 挂载：W_device = W_tcp × inv(T_frame_in_device)，设备根 Follow 法兰，local = T_tool × inv(T_frame_in_device)
/// @param mountTcpWorldForAlign 非空时作为场景世界系 W_tcp；否则由 URDF FK 或法兰位姿推导
CLOUDSIM_HOST_EXPORT bool mountCustomDeviceToFlange(CustomDeviceBackendData& device, DocumentHost& host,
													const QString& robotSceneBackendId, const QString& flangeLinkName,
													const QString& flangeBackendId, const QString& mountFrameBackendId,
													const BackendMat4& toolFrameInFlange,
													const QVector<double>* localJointAnglesRadForMount,
													const BackendMat4* mountTcpWorldForAlign, QString* err);

CLOUDSIM_HOST_EXPORT bool unmountCustomDeviceFromRobot(CustomDeviceBackendData& device, DocumentHost& host,
													   QString* err);

/// 机器人 FK / Follow 求解后：对 Follow 目标为运动学连杆的自定义设备 applyQ
CLOUDSIM_HOST_EXPORT void refreshCustomDevicesFollowingKinematicsTargets(DocumentHost& host);

/// 激活工具系变更后重算已挂设备根 Follow local
CLOUDSIM_HOST_EXPORT void rebakeMountedCustomDevicesFollowLocals(DocumentHost& host);

/// 安装坐标系位姿被用户修改：更新 frameInDeviceW0 并重烘焙设备根 Follow local
CLOUDSIM_HOST_EXPORT bool rebakeMountedDeviceFromInstallFramePose(DocumentHost& host, const std::string& frameBackendId);

/// @deprecated 使用 refreshCustomDevicesFollowingKinematicsTargets
CLOUDSIM_HOST_EXPORT void refreshMountedCustomDevicesAfterRobotFk(DocumentHost& host);

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_CUSTOMDEVICEROBOTMOUNTOPS_H
