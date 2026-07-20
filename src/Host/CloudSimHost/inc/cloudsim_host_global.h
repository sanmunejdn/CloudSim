#ifndef CLOUDSIMHOST_CLOUDSIM_HOST_GLOBAL_H
#define CLOUDSIMHOST_CLOUDSIM_HOST_GLOBAL_H

/// @file cloudsim_host_global.h
/// @brief Host DLL 导出宏

#include <QtCore/qglobal.h>

/// Host DLL 导出宏
#if defined(CLOUDSIM_HOST_LIB)
#define CLOUDSIM_HOST_EXPORT Q_DECL_EXPORT
#else
#define CLOUDSIM_HOST_EXPORT Q_DECL_IMPORT
#endif

#endif // CLOUDSIMHOST_CLOUDSIM_HOST_GLOBAL_H
