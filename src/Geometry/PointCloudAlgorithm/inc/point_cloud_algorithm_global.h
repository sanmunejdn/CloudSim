#pragma once

#if defined(POINT_CLOUD_ALGORITHM_STATIC) || !defined(POINT_CLOUD_ALGORITHM_EXPORTS)
#define POINT_CLOUD_ALGORITHM_API
#else
#if defined(_WIN32)
#ifdef POINT_CLOUD_ALGORITHM_EXPORTS
#define POINT_CLOUD_ALGORITHM_API __declspec(dllexport)
#else
#define POINT_CLOUD_ALGORITHM_API __declspec(dllimport)
#endif
#else
#define POINT_CLOUD_ALGORITHM_API __attribute__((visibility("default")))
#endif
#endif
