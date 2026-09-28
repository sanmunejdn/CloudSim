#ifndef CLOUDSIMPLUGINSDK_IPLUGINROBOTCONTEXT_H
#define CLOUDSIMPLUGINSDK_IPLUGINROBOTCONTEXT_H

/// @file IPluginRobotContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器人窄接口（IPluginHostContext 按域拆分）

/// vtable 仅末尾追加，勿插入中间以免破坏旧插件 ABI

#include "cloudsim_plugin_sdk_global.h"

class IPluginRobotHost;

/// 机器人上下文：机器人运动宿主访问器
class IPluginRobotContext
{
public:
	virtual ~IPluginRobotContext() = default;

	/// 接口版本（0x00010000 = 1.0.0）
	virtual unsigned int contextVersion() const = 0;

	/// 机器人运动宿主（轨迹规划/读 TCP）
	virtual IPluginRobotHost* robotHost() = 0;
	virtual const IPluginRobotHost* robotHost() const = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINROBOTCONTEXT_H
