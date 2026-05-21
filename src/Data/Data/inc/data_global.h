#ifndef _POINTCLOUDPROCESS_DATA_GLOBAL_H_
#define _POINTCLOUDPROCESS_DATA_GLOBAL_H_

// Data DLL export macro.

#if defined(BUILD_STATIC)
# define DATA_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
# if defined(DATA_LIB)
#  define DATA_EXPORT __declspec(dllexport)
# else
#  define DATA_EXPORT __declspec(dllimport)
# endif
#else
# define DATA_EXPORT
#endif

#endif //_POINTCLOUDPROCESS_DATA_GLOBAL_H_