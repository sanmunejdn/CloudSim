#pragma once

#if defined(TRAJECTORY_ALGORITHM_STATIC) || defined(BUILD_STATIC)
#	define TRAJECTORY_ALGORITHM_API
#elif defined(_WIN32) || defined(_WIN64)
#	if defined(TRAJECTORY_ALGORITHM_LIB)
#		define TRAJECTORY_ALGORITHM_API __declspec(dllexport)
#	else
#		define TRAJECTORY_ALGORITHM_API __declspec(dllimport)
#	endif
#else
#	define TRAJECTORY_ALGORITHM_API
#endif

#if defined(TRAJECTORY_ALGORITHM_BUILTINS_STATIC) || defined(BUILD_STATIC)
#	define TRAJECTORY_ALGORITHM_BUILTINS_API
#elif defined(_WIN32) || defined(_WIN64)
#	if defined(TRAJECTORY_ALGORITHM_BUILTINS_LIB)
#		define TRAJECTORY_ALGORITHM_BUILTINS_API __declspec(dllexport)
#	else
#		define TRAJECTORY_ALGORITHM_BUILTINS_API __declspec(dllimport)
#	endif
#else
#	define TRAJECTORY_ALGORITHM_BUILTINS_API
#endif
