#ifndef CLOUDSIMBOOTSTRAP_CLOUDSIM_BOOTSTRAP_GLOBAL_H
#define CLOUDSIMBOOTSTRAP_CLOUDSIM_BOOTSTRAP_GLOBAL_H

/// @file cloudsim_bootstrap_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief CloudSimBootstrap 导出宏

#if defined(CLOUDSIM_BOOTSTRAP_LIB)
#define BOOTSTRAP_EXPORT __declspec(dllexport)
#else
#define BOOTSTRAP_EXPORT __declspec(dllimport)
#endif

#endif // CLOUDSIMBOOTSTRAP_CLOUDSIM_BOOTSTRAP_GLOBAL_H
