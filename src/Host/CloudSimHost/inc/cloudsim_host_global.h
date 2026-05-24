#pragma once

#include <QtCore/qglobal.h>

/// CloudSimHost.dll 符号导出；本工程定义 CLOUDSIM_HOST_LIB 为 export，消费方为 import
#if defined(CLOUDSIM_HOST_LIB)
# define CLOUDSIM_HOST_EXPORT Q_DECL_EXPORT
#else
# define CLOUDSIM_HOST_EXPORT Q_DECL_IMPORT
#endif
