#ifndef ROBOTPATHPLANNING_ROBOT_PATH_PLANNING_GLOBAL_H
#define ROBOTPATHPLANNING_ROBOT_PATH_PLANNING_GLOBAL_H

/// @file robot_path_planning_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RobotPathPlanning 导出宏

#if defined(ROBOT_PATH_PLANNING_STATIC) || defined(BUILD_STATIC)
#define ROBOT_PATH_PLANNING_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(ROBOT_PATH_PLANNING_LIB)
#define ROBOT_PATH_PLANNING_API __declspec(dllexport)
#else
#define ROBOT_PATH_PLANNING_API __declspec(dllimport)
#endif
#else
#define ROBOT_PATH_PLANNING_API
#endif

#endif // ROBOTPATHPLANNING_ROBOT_PATH_PLANNING_GLOBAL_H
