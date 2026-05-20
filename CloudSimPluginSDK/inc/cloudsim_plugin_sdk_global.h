#pragma once

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
# define PLUGIN_SDK_EXPORT
#elif defined(PLUGINSKD_LIB)
# define PLUGIN_SDK_EXPORT Q_DECL_EXPORT
#else
# define PLUGIN_SDK_EXPORT Q_DECL_IMPORT
#endif

/// Host ABI version (major in high 16 bits, minor in low 16 bits). Must match \ref cloudsimPluginHostVersion().
#define CLOUDSIM_PLUGIN_HOST_VERSION 0x00010100
