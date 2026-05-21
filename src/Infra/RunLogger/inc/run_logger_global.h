#pragma once

#if defined(RUN_LOGGER_STATIC) || defined(BUILD_STATIC)
#	define RUN_LOGGER_API
#elif defined(_WIN32) || defined(_WIN64)
#	if defined(RUN_LOGGER_LIB)
#		define RUN_LOGGER_API __declspec(dllexport)
#	else
#		define RUN_LOGGER_API __declspec(dllimport)
#	endif
#else
#	define RUN_LOGGER_API
#endif
