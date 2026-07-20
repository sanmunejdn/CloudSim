#ifndef DATA_DATA_GLOBAL_H
#define DATA_DATA_GLOBAL_H

/// @file data_global.h
/// @brief Data 模块 DLL 导出宏

/// Data 模块 DLL 导出宏

#if defined(BUILD_STATIC)
#define DATA_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(DATA_LIB)
#define DATA_EXPORT __declspec(dllexport)
#else
#define DATA_EXPORT __declspec(dllimport)
#endif
#else
#define DATA_EXPORT
#endif

#endif // DATA_DATA_GLOBAL_H
