#pragma once

#if defined(VCg_ALGORITHMS_STATIC) || defined(BUILD_STATIC)
#	define VCg_ALGORITHMS_API
#elif defined(_WIN32) || defined(_WIN64)
#	if defined(VCg_ALGORITHMS_LIB)
#		define VCg_ALGORITHMS_API __declspec(dllexport)
#	else
#		define VCg_ALGORITHMS_API __declspec(dllimport)
#	endif
#else
#	define VCg_ALGORITHMS_API
#endif
