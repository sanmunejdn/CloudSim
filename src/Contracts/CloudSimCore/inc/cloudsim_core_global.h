#ifndef CLOUDSIMCORE_CLOUDSIM_CORE_GLOBAL_H
#define CLOUDSIMCORE_CLOUDSIM_CORE_GLOBAL_H

/// @file cloudsim_core_global.h
/// @brief API 版本（高16主版本）

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
#define CLOUDSIM_CORE_EXPORT
#elif defined(CLOUDSIM_CORE_LIB)
#define CLOUDSIM_CORE_EXPORT Q_DECL_EXPORT
#else
#define CLOUDSIM_CORE_EXPORT Q_DECL_IMPORT
#endif

/// API 版本（高16主版本）
#define CLOUDSIM_CORE_API_VERSION 0x00010000

#endif // CLOUDSIMCORE_CLOUDSIM_CORE_GLOBAL_H
