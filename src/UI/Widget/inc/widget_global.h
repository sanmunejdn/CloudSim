#ifndef WIDGET_WIDGET_GLOBAL_H
#define WIDGET_WIDGET_GLOBAL_H

/// @file widget_global.h
/// @brief Widget 导出宏与 UTF-8 执行字符集

/// Widget 导出宏与 UTF-8 执行字符集

#include <QtCore/qglobal.h>

#if defined(_WIN64) || defined(_WIN32)
#pragma execution_character_set("utf-8")
#endif

#ifndef BUILD_STATIC
#if defined(WIDGET_LIB)
#define WIDGET_EXPORT Q_DECL_EXPORT
#else
#define WIDGET_EXPORT Q_DECL_IMPORT
#endif
#else
#define WIDGET_EXPORT
#endif

#if defined(CLOUDSIM_OSG_IN_HOST)
#include "../../Host/CloudSimHost/inc/cloudsim_host_global.h"
#define OSG_WIDGET_API CLOUDSIM_HOST_EXPORT
#else
#define OSG_WIDGET_API WIDGET_EXPORT
#endif

#define HPL_TRY \
	try         \
	{
#define HPL_CATCH                                                    \
	}                                                                \
	catch (const std::exception& exc) { qCritical() << exc.what(); } \
	catch (...) { qCritical() << "空指针、野指针：" << __FILE__ << ", " << __func__ << ", " << __LINE__; }

#endif // WIDGET_WIDGET_GLOBAL_H
