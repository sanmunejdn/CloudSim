#ifndef KINEMATICCORE_KINEMATIC_CORE_GLOBAL_H
#define KINEMATICCORE_KINEMATIC_CORE_GLOBAL_H

/// @file kinematic_core_global.h
/// @brief KinematicCore DLL 导出宏

#if defined(KINEMATIC_CORE_STATIC) || defined(BUILD_STATIC)
#define KINEMATIC_CORE_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(KINEMATIC_CORE_LIB)
#define KINEMATIC_CORE_API __declspec(dllexport)
#else
#define KINEMATIC_CORE_API __declspec(dllimport)
#endif
#else
#define KINEMATIC_CORE_API
#endif

#define KINEMATIC_CORE_API_VERSION 0x00010000

#endif // KINEMATICCORE_KINEMATIC_CORE_GLOBAL_H
