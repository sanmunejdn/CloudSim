#ifndef CLOUDSIMLABELINGSDK_LABELING_SDK_GLOBAL_H
#define CLOUDSIMLABELINGSDK_LABELING_SDK_GLOBAL_H

/// @file labeling_sdk_global.h
/// @brief CloudSimLabelingSDK 导出宏

#if defined(LABELING_SDK_STATIC) || defined(BUILD_STATIC)
#define LABELING_SDK_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(LABELING_SDK_LIB)
#define LABELING_SDK_EXPORT __declspec(dllexport)
#else
#define LABELING_SDK_EXPORT __declspec(dllimport)
#endif
#else
#define LABELING_SDK_EXPORT
#endif

#define LABELING_SDK_VERSION 0x00010000

#endif // CLOUDSIMLABELINGSDK_LABELING_SDK_GLOBAL_H
