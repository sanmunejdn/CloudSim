#ifndef CLOUDSIMAISDK_CLOUDSIM_AI_SDK_GLOBAL_H
#define CLOUDSIMAISDK_CLOUDSIM_AI_SDK_GLOBAL_H

/// @file cloudsim_ai_sdk_global.h
/// @brief AiSDK ABI 版本（高 16 主、低 16 次）

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
#define CLOUDSIM_AI_SDK_EXPORT
#elif defined(CLOUDSIM_AI_SDK_LIB)
#define CLOUDSIM_AI_SDK_EXPORT Q_DECL_EXPORT
#else
#define CLOUDSIM_AI_SDK_EXPORT Q_DECL_IMPORT
#endif

/// AiSDK ABI 版本（高 16 主、低 16 次）
#define CLOUDSIM_AI_SDK_VERSION 0x00010000

#endif // CLOUDSIMAISDK_CLOUDSIM_AI_SDK_GLOBAL_H
