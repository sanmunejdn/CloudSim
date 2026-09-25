#ifndef CLOUDSIMPLUGINSDK_PLUGINROBOTTYPES_H
#define CLOUDSIMPLUGINSDK_PLUGINROBOTTYPES_H

/// @file PluginRobotTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 插件侧机器人位姿类型（1.55.0+）

#include "cloudsim_plugin_sdk_global.h"

/// TCP / 物体位姿：平移 mm，欧拉 deg（与示教 TCP 一致）
struct PluginPose6d
{
	double xMm = 0.0;
	double yMm = 0.0;
	double zMm = 0.0;
	double rxDeg = 0.0;
	double ryDeg = 0.0;
	double rzDeg = 0.0;
};

#endif // CLOUDSIMPLUGINSDK_PLUGINROBOTTYPES_H
