#pragma once

/// Host 编 OSG 栈时强制 CLOUDSIM_HOST_LIB，WIDGET_EXPORT/OSG_WIDGET_API 与 Host 同 DLL 导出
#ifndef CLOUDSIM_HOST_LIB
#define CLOUDSIM_HOST_LIB
#endif

#include "cloudsim_host_global.h"

#define WIDGET_EXPORT CLOUDSIM_HOST_EXPORT
#define OSG_WIDGET_API CLOUDSIM_HOST_EXPORT
