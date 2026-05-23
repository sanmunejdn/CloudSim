#pragma once

#if defined(BACKENDVISUAL_STATIC) || defined(BUILD_STATIC)
#	define BACKENDVISUAL_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#	if defined(BACKENDVISUAL_LIB)
#		define BACKENDVISUAL_EXPORT __declspec(dllexport)
#	else
#		define BACKENDVISUAL_EXPORT __declspec(dllimport)
#	endif
#else
#	define BACKENDVISUAL_EXPORT
#endif
