#ifndef PROGRESS_PROGRESS_GLOBAL_H
#define PROGRESS_PROGRESS_GLOBAL_H

/// @file progress_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Progress 导出宏

#if defined(PROGRESS_STATIC) || defined(BUILD_STATIC)
#define PROGRESS_API
#elif defined(_WIN32) || defined(_WIN64)
#if defined(PROGRESS_LIB)
#define PROGRESS_API __declspec(dllexport)
#else
#define PROGRESS_API __declspec(dllimport)
#endif
#else
#define PROGRESS_API
#endif

#endif // PROGRESS_PROGRESS_GLOBAL_H
