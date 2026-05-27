#pragma once

#if defined(PLCCOMM_SDK_STATIC) || defined(BUILD_STATIC)
# define PLCCOMM_SDK_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
# if defined(PLCCOMM_SDK_LIB)
#  define PLCCOMM_SDK_EXPORT __declspec(dllexport)
# else
#  define PLCCOMM_SDK_EXPORT __declspec(dllimport)
# endif
#else
# define PLCCOMM_SDK_EXPORT
#endif

/// PlcCommSDK ABI 版本（高 16 主、低 16 次）
#define PLCCOMM_SDK_VERSION 0x00010000
