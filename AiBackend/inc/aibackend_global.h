#ifndef AIBACKEND_GLOBAL_H
#define AIBACKEND_GLOBAL_H

#if defined(BUILD_STATIC)
# define AIBACKEND_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
# if defined(AIBACKEND_LIB)
#  define AIBACKEND_EXPORT __declspec(dllexport)
# else
#  define AIBACKEND_EXPORT __declspec(dllimport)
# endif
#else
# define AIBACKEND_EXPORT
#endif

#endif
