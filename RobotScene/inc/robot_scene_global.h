#pragma once

#if defined(ROBOT_SCENE_STATIC) || defined(BUILD_STATIC)
#	define ROBOT_SCENE_API
#elif defined(_WIN32) || defined(_WIN64)
#	if defined(ROBOT_SCENE_LIB)
#		define ROBOT_SCENE_API __declspec(dllexport)
#	else
#		define ROBOT_SCENE_API __declspec(dllimport)
#	endif
#else
#	define ROBOT_SCENE_API
#endif
