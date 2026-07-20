#ifndef ROBOTURDF_ROBOT_URDF_GLOBAL_H
#define ROBOTURDF_ROBOT_URDF_GLOBAL_H

/// @file robot_urdf_global.h
/// @brief RobotUrdf 导出宏

#if defined(ROBOT_URDF_STATIC) || defined(BUILD_STATIC)
#define ROBOT_URDF_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(ROBOT_URDF_LIB)
#define ROBOT_URDF_API __declspec(dllexport)
#else
#define ROBOT_URDF_API __declspec(dllimport)
#endif
#else
#define ROBOT_URDF_API
#endif

#endif // ROBOTURDF_ROBOT_URDF_GLOBAL_H
