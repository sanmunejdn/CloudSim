#ifndef ROBOTSCENE_ROBOT_SCENE_GLOBAL_H
#define ROBOTSCENE_ROBOT_SCENE_GLOBAL_H

/// @file robot_scene_global.h
/// @brief RobotScene 导出宏

#if defined(ROBOT_SCENE_STATIC) || defined(BUILD_STATIC)
#define ROBOT_SCENE_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(ROBOT_SCENE_LIB)
#define ROBOT_SCENE_API __declspec(dllexport)
#else
#define ROBOT_SCENE_API __declspec(dllimport)
#endif
#else
#define ROBOT_SCENE_API
#endif

#endif // ROBOTSCENE_ROBOT_SCENE_GLOBAL_H
