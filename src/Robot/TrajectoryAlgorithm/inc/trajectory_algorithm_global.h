#ifndef TRAJECTORYALGORITHM_TRAJECTORY_ALGORITHM_GLOBAL_H
#define TRAJECTORYALGORITHM_TRAJECTORY_ALGORITHM_GLOBAL_H

/// @file trajectory_algorithm_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief TrajectoryAlgorithm 导出宏

#if defined(TRAJECTORY_ALGORITHM_STATIC) || defined(BUILD_STATIC)
#define TRAJECTORY_ALGORITHM_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(TRAJECTORY_ALGORITHM_LIB)
#define TRAJECTORY_ALGORITHM_API __declspec(dllexport)
#else
#define TRAJECTORY_ALGORITHM_API __declspec(dllimport)
#endif
#else
#define TRAJECTORY_ALGORITHM_API
#endif

#if defined(TRAJECTORY_ALGORITHM_BUILTINS_STATIC) || defined(BUILD_STATIC)
#define TRAJECTORY_ALGORITHM_BUILTINS_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(TRAJECTORY_ALGORITHM_BUILTINS_LIB)
#define TRAJECTORY_ALGORITHM_BUILTINS_API __declspec(dllexport)
#else
#define TRAJECTORY_ALGORITHM_BUILTINS_API __declspec(dllimport)
#endif
#else
#define TRAJECTORY_ALGORITHM_BUILTINS_API
#endif

#endif // TRAJECTORYALGORITHM_TRAJECTORY_ALGORITHM_GLOBAL_H
