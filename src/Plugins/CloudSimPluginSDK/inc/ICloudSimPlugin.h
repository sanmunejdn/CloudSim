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

#define CloudSimPlugin_iid "com.cloudsim.ICloudSimPlugin/1.0"
Q_DECLARE_INTERFACE(ICloudSimPlugin, CloudSimPlugin_iid)

#endif // CLOUDSIMPLUGINSDK_ICLOUDSIMPLUGIN_H
