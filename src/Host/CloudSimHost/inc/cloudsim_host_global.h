#pragma once

#include <QtCore/qglobal.h>

/// Host DLL 导出宏
#if defined(CLOUDSIM_HOST_LIB)
# define CLOUDSIM_HOST_EXPORT Q_DECL_EXPORT
#else
# define CLOUDSIM_HOST_EXPORT Q_DECL_IMPORT
#endif
