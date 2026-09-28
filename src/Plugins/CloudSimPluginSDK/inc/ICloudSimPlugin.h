#ifndef CLOUDSIMPLUGINSDK_ICLOUDSIMPLUGIN_H
#define CLOUDSIMPLUGINSDK_ICLOUDSIMPLUGIN_H

/// @file ICloudSimPlugin.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief CloudSim 动态插件 DLL 契约

#include "cloudsim_plugin_sdk_global.h"

#include <QString>
#include <QtPlugin>

class IPluginHostContext;

/// CloudSim 动态插件 DLL 契约
class ICloudSimPlugin
{
public:
	virtual ~ICloudSimPlugin() = default;

	virtual QString pluginId() const = 0;
	virtual QString displayName() const = 0;

	/// UI 线程 manifest 校验后；false 跳过插件
	virtual bool initialize(IPluginHostContext* host) = 0;

	/// 退出前 UI 线程回调；运行时不会卸载插件
	virtual void shutdown() = 0;
};

/// 插件 ABI 版本编码进 IID：宿主与插件编译自同一 SDK 头即自动一致；
/// 旧插件 IID 无版本后缀，qobject_cast 直接失败，宿主经 metaData 读取 IID 给出告警
#define CloudSimPlugin_iid "com.cloudsim.ICloudSimPlugin/1.0." CLOUDSIM_PLUGIN_SDK_VERSION_STR
Q_DECLARE_INTERFACE(ICloudSimPlugin, CloudSimPlugin_iid)

#endif // CLOUDSIMPLUGINSDK_ICLOUDSIMPLUGIN_H
