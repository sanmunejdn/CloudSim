#pragma once

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
# define CLOUDSIM_CORE_EXPORT
#elif defined(CLOUDSIM_CORE_LIB)
# define CLOUDSIM_CORE_EXPORT Q_DECL_EXPORT
#else
# define CLOUDSIM_CORE_EXPORT Q_DECL_IMPORT
#endif

/// API version: major in high 16 bits, minor in low 16 bits.
#define CLOUDSIM_CORE_API_VERSION 0x00010000
