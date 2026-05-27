#pragma once

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
# define PLUGIN_SDK_EXPORT
#elif defined(PLUGINSKD_LIB)
# define PLUGIN_SDK_EXPORT Q_DECL_EXPORT
#else
# define PLUGIN_SDK_EXPORT Q_DECL_IMPORT
#endif

/// 宿主 ABI 版本（高 16 主、低 16 次），须与 cloudsimPluginHostVersion 一致
#define CLOUDSIM_PLUGIN_HOST_VERSION 0x00010300
