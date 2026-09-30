#ifndef CLOUDSIMHOST_WIDGET_GLOBAL_H
#define CLOUDSIMHOST_WIDGET_GLOBAL_H

/// @file widget_global.h
/// @brief Host 编入 OSG/Plugin 源时的导出宏（须先于 Widget 侧同名头被找到）

#include <QtCore/qglobal.h>

#include "cloudsim_host_global.h"
#include "cloudsim_viewport_global.h"

#if defined(_WIN64) || defined(_WIN32)
#pragma execution_character_set("utf-8")
#endif

// Host 工程同时定义 WIDGET_LIB：迁入的 Widget 源仍走 WIDGET_EXPORT
#ifndef BUILD_STATIC
#if defined(WIDGET_LIB) || defined(CLOUDSIM_HOST_LIB)
#define WIDGET_EXPORT Q_DECL_EXPORT
#else
#define WIDGET_EXPORT Q_DECL_IMPORT
#endif
#else
#define WIDGET_EXPORT
#endif

// 桌面 OsgWidget 等在 Viewport.dll；Headless 桩仍走 HOST_EXPORT
#ifndef OSG_WIDGET_API
#if defined(CLOUDSIM_HOST_HEADLESS_ONLY)
#define OSG_WIDGET_API CLOUDSIM_HOST_EXPORT
#else
#define OSG_WIDGET_API CLOUDSIM_VIEWPORT_EXPORT
#endif
#endif

#define HPL_TRY \
	try         \
	{
#define HPL_CATCH                                                    \
	}                                                                \
	catch (const std::exception& exc) { qCritical() << exc.what(); } \
	catch (...) { qCritical() << "空指针、野指针：" << __FILE__ << ", " << __func__ << ", " << __LINE__; }

#endif // CLOUDSIMHOST_WIDGET_GLOBAL_H
