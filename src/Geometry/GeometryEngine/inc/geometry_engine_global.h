#ifndef GEOMETRYENGINE_GEOMETRY_ENGINE_GLOBAL_H
#define GEOMETRYENGINE_GEOMETRY_ENGINE_GLOBAL_H

/// @file geometry_engine_global.h
/// @brief GeometryEngine 导出宏

#if defined(GEOMETRY_ENGINE_STATIC) || defined(BUILD_STATIC)
#define GEOMETRY_ENGINE_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(GEOMETRY_ENGINE_LIB)
#define GEOMETRY_ENGINE_API __declspec(dllexport)
#else
#define GEOMETRY_ENGINE_API __declspec(dllimport)
#endif
#else
#define GEOMETRY_ENGINE_API
#endif

#endif // GEOMETRYENGINE_GEOMETRY_ENGINE_GLOBAL_H
