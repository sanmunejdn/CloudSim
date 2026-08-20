#ifndef INDUSTRIALCAMERASDK_INDUSTRIAL_CAMERA_SDK_GLOBAL_H
#define INDUSTRIALCAMERASDK_INDUSTRIAL_CAMERA_SDK_GLOBAL_H

/// @file industrial_camera_sdk_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief IndustrialCameraSDK 导出与 ABI 版本

#if defined(INDUSTRIAL_CAMERA_SDK_STATIC) || defined(BUILD_STATIC)
#define INDUSTRIAL_CAMERA_SDK_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(INDUSTRIAL_CAMERA_SDK_LIB)
#define INDUSTRIAL_CAMERA_SDK_EXPORT __declspec(dllexport)
#else
#define INDUSTRIAL_CAMERA_SDK_EXPORT __declspec(dllimport)
#endif
#else
#define INDUSTRIAL_CAMERA_SDK_EXPORT
#endif

#define INDUSTRIAL_CAMERA_SDK_VERSION 0x00010000

#endif // INDUSTRIALCAMERASDK_INDUSTRIAL_CAMERA_SDK_GLOBAL_H
