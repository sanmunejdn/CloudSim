#ifndef BACKENDVISUAL_BACKENDVISUAL_GLOBAL_H
#define BACKENDVISUAL_BACKENDVISUAL_GLOBAL_H

/// @file backendvisual_global.h
/// @brief BackendVisual 导出宏

#if defined(BACKENDVISUAL_STATIC) || defined(BUILD_STATIC)
#define BACKENDVISUAL_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(BACKENDVISUAL_LIB)
#define BACKENDVISUAL_EXPORT __declspec(dllexport)
#else
#define BACKENDVISUAL_EXPORT __declspec(dllimport)
#endif
#else
#define BACKENDVISUAL_EXPORT
#endif

#endif // BACKENDVISUAL_BACKENDVISUAL_GLOBAL_H
