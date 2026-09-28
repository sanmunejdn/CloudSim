#ifndef CLOUDSIMHOST_CLOUDSIM_HOST_GLOBAL_H
#define CLOUDSIMHOST_CLOUDSIM_HOST_GLOBAL_H

/// @file cloudsim_host_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Host DLL 导出宏

#include <QtCore/qglobal.h>

/// Host 导出宏：静态库消费方可定义 CLOUDSIM_HOST_STATIC；
/// HostCore 链入 DLL 时用 CLOUDSIM_HOST_LIB（dllexport），并由 DLL 以 /WHOLEARCHIVE 再导出给 Widget 等
#if defined(CLOUDSIM_HOST_STATIC)
#define CLOUDSIM_HOST_EXPORT
#elif defined(CLOUDSIM_HOST_LIB)
#define CLOUDSIM_HOST_EXPORT Q_DECL_EXPORT
#else
#define CLOUDSIM_HOST_EXPORT Q_DECL_IMPORT
#endif

#endif // CLOUDSIMHOST_CLOUDSIM_HOST_GLOBAL_H
