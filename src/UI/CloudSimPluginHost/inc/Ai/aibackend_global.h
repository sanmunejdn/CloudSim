#ifndef CLOUDSIMPLUGINHOST_AIBACKEND_GLOBAL_H
#define CLOUDSIMPLUGINHOST_AIBACKEND_GLOBAL_H

/// @file aibackend_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief CloudSimPluginHost 导出宏

#if defined(BUILD_STATIC)
#define AIBACKEND_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(AIBACKEND_LIB)
#define AIBACKEND_EXPORT __declspec(dllexport)
#else
#define AIBACKEND_EXPORT __declspec(dllimport)
#endif
#else
#define AIBACKEND_EXPORT
#endif

#endif // CLOUDSIMPLUGINHOST_AIBACKEND_GLOBAL_H
