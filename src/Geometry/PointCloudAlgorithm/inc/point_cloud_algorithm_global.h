#ifndef POINTCLOUDALGORITHM_POINT_CLOUD_ALGORITHM_GLOBAL_H
#define POINTCLOUDALGORITHM_POINT_CLOUD_ALGORITHM_GLOBAL_H

/// @file point_cloud_algorithm_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PointCloudAlgorithm 导出宏

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

#endif // POINTCLOUDALGORITHM_POINT_CLOUD_ALGORITHM_GLOBAL_H
