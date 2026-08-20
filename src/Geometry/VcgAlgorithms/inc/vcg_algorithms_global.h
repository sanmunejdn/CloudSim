#ifndef VCGALGORITHMS_VCG_ALGORITHMS_GLOBAL_H
#define VCGALGORITHMS_VCG_ALGORITHMS_GLOBAL_H

/// @file vcg_algorithms_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief VcgAlgorithms 导出宏

#if defined(VCg_ALGORITHMS_STATIC) || defined(BUILD_STATIC)
#define VCg_ALGORITHMS_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(VCg_ALGORITHMS_LIB)
#define VCg_ALGORITHMS_API __declspec(dllexport)
#else
#define VCg_ALGORITHMS_API __declspec(dllimport)
#endif
#else
#define VCg_ALGORITHMS_API
#endif

#endif // VCGALGORITHMS_VCG_ALGORITHMS_GLOBAL_H
