#ifndef GEOMETRYALGORITHM_GEOMETRY_ALGORITHM_GLOBAL_H
#define GEOMETRYALGORITHM_GEOMETRY_ALGORITHM_GLOBAL_H

/// @file geometry_algorithm_global.h
/// @brief GeometryAlgorithm 导出宏

#if defined(GEOMETRY_ALGORITHM_STATIC) || defined(BUILD_STATIC)
#define GEOMETRY_ALGORITHM_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(GEOMETRY_ALGORITHM_LIB)
#define GEOMETRY_ALGORITHM_API __declspec(dllexport)
#else
#define GEOMETRY_ALGORITHM_API __declspec(dllimport)
#endif
#else
#define GEOMETRY_ALGORITHM_API
#endif

#endif // GEOMETRYALGORITHM_GEOMETRY_ALGORITHM_GLOBAL_H
