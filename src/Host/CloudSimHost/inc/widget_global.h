#pragma once

// Host 工程编译 OSG 栈时强制导出，避免 MOC 走 Widget 的 dllimport。
#ifndef CLOUDSIM_HOST_LIB
#define CLOUDSIM_HOST_LIB
#endif

#include "cloudsim_host_global.h"

#define WIDGET_EXPORT CLOUDSIM_HOST_EXPORT
#define OSG_WIDGET_API CLOUDSIM_HOST_EXPORT
