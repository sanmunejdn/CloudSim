#pragma once

#if defined(OSGWIDGETCORE_STATIC) || defined(BUILD_STATIC)
#	define OSGWIDGETCORE_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#	if defined(OSGWIDGETCORE_LIB)
#		define OSGWIDGETCORE_EXPORT __declspec(dllexport)
#	else
#		define OSGWIDGETCORE_EXPORT __declspec(dllimport)
#	endif
#else
#	define OSGWIDGETCORE_EXPORT
#endif
