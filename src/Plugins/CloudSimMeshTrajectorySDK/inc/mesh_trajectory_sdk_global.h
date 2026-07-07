#pragma once

#if defined(MESH_TRAJECTORY_SDK_STATIC) || defined(BUILD_STATIC)
# define MESH_TRAJECTORY_SDK_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
# if defined(MESH_TRAJECTORY_SDK_LIB)
#  define MESH_TRAJECTORY_SDK_EXPORT __declspec(dllexport)
# else
#  define MESH_TRAJECTORY_SDK_EXPORT __declspec(dllimport)
# endif
#else
# define MESH_TRAJECTORY_SDK_EXPORT
#endif

#define MESH_TRAJECTORY_SDK_VERSION 0x00010000
