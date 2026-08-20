#ifndef PLCCOMMSDK_PLC_COMM_SDK_GLOBAL_H
#define PLCCOMMSDK_PLC_COMM_SDK_GLOBAL_H

/// @file plc_comm_sdk_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PlcCommSDK ABI 版本（高 16 主、低 16 次）

#if defined(PLCCOMM_SDK_STATIC) || defined(BUILD_STATIC)
#define PLCCOMM_SDK_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(PLCCOMM_SDK_LIB)
#define PLCCOMM_SDK_EXPORT __declspec(dllexport)
#else
#define PLCCOMM_SDK_EXPORT __declspec(dllimport)
#endif
#else
#define PLCCOMM_SDK_EXPORT
#endif

/// PlcCommSDK ABI 版本（高 16 主、低 16 次）
#define PLCCOMM_SDK_VERSION 0x00010000

#endif // PLCCOMMSDK_PLC_COMM_SDK_GLOBAL_H
