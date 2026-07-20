#ifndef ROBOTKINEMATICS_ROBOT_KINEMATICS_GLOBAL_H
#define ROBOTKINEMATICS_ROBOT_KINEMATICS_GLOBAL_H

/// @file robot_kinematics_global.h
/// @brief 静态库：定义 ROBOT_KINEMATICS_STATIC；若将来改为 DLL，再定义 ROBOT_KINEMATICS_LIB 并导出

/// 静态库：定义 ROBOT_KINEMATICS_STATIC；若将来改为 DLL，再定义 ROBOT_KINEMATICS_LIB 并导出

#if defined(ROBOT_KINEMATICS_STATIC) || defined(BUILD_STATIC)
#define ROBOT_KINEMATICS_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(ROBOT_KINEMATICS_LIB)
#define ROBOT_KINEMATICS_API __declspec(dllexport)
#else
#define ROBOT_KINEMATICS_API __declspec(dllimport)
#endif
#else
#define ROBOT_KINEMATICS_API
#endif

#endif // ROBOTKINEMATICS_ROBOT_KINEMATICS_GLOBAL_H
