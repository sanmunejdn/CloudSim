#ifndef CLOUDSIMPLUGINSDK_CLOUDSIM_PLUGIN_SDK_GLOBAL_H
#define CLOUDSIMPLUGINSDK_CLOUDSIM_PLUGIN_SDK_GLOBAL_H

/// @file cloudsim_plugin_sdk_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 宿主 ABI 版本（高 16 主、低 16 次），须与 cloudsimPluginHostVersion 一致

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
#define PLUGIN_SDK_EXPORT
#elif defined(PLUGINSKD_LIB)
#define PLUGIN_SDK_EXPORT Q_DECL_EXPORT
#else
#define PLUGIN_SDK_EXPORT Q_DECL_IMPORT
#endif

/// 宿主 ABI 版本（高 16 主、低 16 次），须与 cloudsimPluginHostVersion 一致
#define CLOUDSIM_PLUGIN_HOST_VERSION 0x00013700

/// 插件编译期 SDK ABI 版本（高 16 主、低 16 次），与 CLOUDSIM_PLUGIN_HOST_VERSION 同步演进
#define CLOUDSIM_PLUGIN_SDK_VERSION 0x00013700

/// SDK 版本字符串形式，用于编码进 IID
#define CLOUDSIM_PLUGIN_SDK_VERSION_STR "0x00013700"

#endif // CLOUDSIMPLUGINSDK_CLOUDSIM_PLUGIN_SDK_GLOBAL_H
