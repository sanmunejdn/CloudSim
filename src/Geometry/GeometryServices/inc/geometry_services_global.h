#ifndef GEOMETRY_SERVICES_GLOBAL_H
#define GEOMETRY_SERVICES_GLOBAL_H

/// @file geometry_services_global.h
/// @brief GeometryServices 模块 DLL 导出宏

#if defined(BUILD_STATIC)
#define GEOMETRY_SERVICES_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(GEOMETRY_SERVICES_LIB)
#define GEOMETRY_SERVICES_EXPORT __declspec(dllexport)
#else
#define GEOMETRY_SERVICES_EXPORT __declspec(dllimport)
#endif
#else
#define GEOMETRY_SERVICES_EXPORT
#endif

#endif // GEOMETRY_SERVICES_GLOBAL_H
