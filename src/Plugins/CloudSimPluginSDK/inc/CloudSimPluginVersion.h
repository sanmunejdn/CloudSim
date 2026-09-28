#ifndef CLOUDSIMPLUGINSDK_CLOUDSIMPLUGINVERSION_H
#define CLOUDSIMPLUGINSDK_CLOUDSIMPLUGINVERSION_H

/// @file CloudSimPluginVersion.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief CloudSimPluginVersion 接口

#include "cloudsim_plugin_sdk_global.h"

PLUGIN_SDK_EXPORT unsigned int cloudsimPluginHostVersion();

/// 插件编译期 SDK ABI 版本（编译进插件 DLL 的常量，供宿主握手）
PLUGIN_SDK_EXPORT unsigned int cloudsimPluginSdkVersion();

#endif // CLOUDSIMPLUGINSDK_CLOUDSIMPLUGINVERSION_H
