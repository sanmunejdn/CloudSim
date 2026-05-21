#pragma once

#if defined(ROBOT_URDF_STATIC) || defined(BUILD_STATIC)
#	define ROBOT_URDF_API
#elif defined(_WIN32) || defined(_WIN64)
#	if defined(ROBOT_URDF_LIB)
#		define ROBOT_URDF_API __declspec(dllexport)
#	else
#		define ROBOT_URDF_API __declspec(dllimport)
#	endif
#else
#	define ROBOT_URDF_API
#endif
