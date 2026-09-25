#ifndef CLOUDSIMPLUGINSDK_IPLUGINROBOTHOST_H
#define CLOUDSIMPLUGINSDK_IPLUGINROBOTHOST_H

/// @file IPluginRobotHost.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人运动宿主 API（1.55.0+）；经 IPluginHostContext::robotHost() 获取

#include "cloudsim_plugin_sdk_global.h"

#include "PluginRobotTypes.h"

#include <QString>
#include <QVector>

/// 机器人运动宿主（1.55.0+）；无仿真上下文时方法返回 false
class IPluginRobotHost
{
public:
	virtual ~IPluginRobotHost() = default;

	/// 活动机器人 TCP（基座系）
	virtual bool getActiveRobotTcpPose(PluginPose6d& outMmDeg, QString* err) = 0;

	/// 选中对象世界位姿 → 活动机器人基座系
	virtual bool getSelectedBackendWorldPose(PluginPose6d& outMmDeg, QString* err) = 0;

	/// 按段 planToTcpPose（碰撞页参数），预览确认后写入活动程序
	virtual bool planAndConfirmTcpWaypoints(const QVector<PluginPose6d>& goalsMmDeg, QString* err) = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINROBOTHOST_H
