#ifndef COLLISIONALGORITHM_COLLISION_ALGORITHM_GLOBAL_H
#define COLLISIONALGORITHM_COLLISION_ALGORITHM_GLOBAL_H

/// @file collision_algorithm_global.h
/// @brief CollisionAlgorithm 导出宏

#if defined(COLLISION_ALGORITHM_STATIC) || defined(BUILD_STATIC)
#define COLLISION_ALGORITHM_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(COLLISION_ALGORITHM_LIB)
#define COLLISION_ALGORITHM_API __declspec(dllexport)
#else
#define COLLISION_ALGORITHM_API __declspec(dllimport)
#endif
#else
#define COLLISION_ALGORITHM_API
#endif

#endif // COLLISIONALGORITHM_COLLISION_ALGORITHM_GLOBAL_H
