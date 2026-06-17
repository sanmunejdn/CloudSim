#pragma once

#if defined(LABELING_SDK_STATIC) || defined(BUILD_STATIC)
# define LABELING_SDK_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
# if defined(LABELING_SDK_LIB)
#  define LABELING_SDK_EXPORT __declspec(dllexport)
# else
#  define LABELING_SDK_EXPORT __declspec(dllimport)
# endif
#else
# define LABELING_SDK_EXPORT
#endif

#define LABELING_SDK_VERSION 0x00010000
