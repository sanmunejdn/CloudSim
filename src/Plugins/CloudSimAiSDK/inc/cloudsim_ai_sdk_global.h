#ifndef CLOUDSIMAISDK_CLOUDSIM_AI_SDK_GLOBAL_H
#define CLOUDSIMAISDK_CLOUDSIM_AI_SDK_GLOBAL_H

/// @file cloudsim_ai_sdk_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AiSDK ABI 版本（高 16 主、低 16 次）

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
#define CLOUDSIM_AI_SDK_EXPORT
#elif defined(CLOUDSIM_AI_SDK_LIB)
#define CLOUDSIM_AI_SDK_EXPORT Q_DECL_EXPORT
#else
#define CLOUDSIM_AI_SDK_EXPORT Q_DECL_IMPORT
#endif

/// AiSDK ABI 版本（高 16 主、低 16 次）；与核心 PluginSDK 同步演进，降低版本矩阵
#define CLOUDSIM_AI_SDK_VERSION 0x00013A00

/// 字符串形式，须与 CloudSimAiPlugin_iid 字面量后缀一致
#define CLOUDSIM_AI_SDK_VERSION_STR "0x00013A00"

#endif // CLOUDSIMAISDK_CLOUDSIM_AI_SDK_GLOBAL_H
