#ifndef ROBOTWIDGET_ROBOTWIDGET_GLOBAL_H
#define ROBOTWIDGET_ROBOTWIDGET_GLOBAL_H

/// @file robotwidget_global.h
/// @brief RobotWidget 导出宏

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
#define ROBOTWIDGET_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(ROBOTWIDGET_LIB)
#define ROBOTWIDGET_EXPORT Q_DECL_EXPORT
#else
#define ROBOTWIDGET_EXPORT Q_DECL_IMPORT
#endif
#else
#define ROBOTWIDGET_EXPORT
#endif

#endif // ROBOTWIDGET_ROBOTWIDGET_GLOBAL_H
