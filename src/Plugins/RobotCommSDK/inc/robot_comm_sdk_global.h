#ifndef ROBOTCOMMSDK_ROBOT_COMM_SDK_GLOBAL_H
#define ROBOTCOMMSDK_ROBOT_COMM_SDK_GLOBAL_H

/// @file robot_comm_sdk_global.h
/// @brief RobotCommSDK 导出宏与 ABI 版本

#if defined(ROBOTCOMM_SDK_STATIC) || defined(BUILD_STATIC)
#define ROBOTCOMM_SDK_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(ROBOTCOMM_SDK_LIB)
#define ROBOTCOMM_SDK_EXPORT __declspec(dllexport)
#else
#define ROBOTCOMM_SDK_EXPORT __declspec(dllimport)
#endif
#else
#define ROBOTCOMM_SDK_EXPORT
#endif

#define ROBOTCOMM_SDK_VERSION 0x00010000

#endif // ROBOTCOMMSDK_ROBOT_COMM_SDK_GLOBAL_H
