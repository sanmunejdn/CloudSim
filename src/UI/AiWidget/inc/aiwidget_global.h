#ifndef AIWIDGET_AIWIDGET_GLOBAL_H
#define AIWIDGET_AIWIDGET_GLOBAL_H

/// @file aiwidget_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AiWidget 导出宏

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
#define AIWIDGET_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
#if defined(AIWIDGET_LIB)
#define AIWIDGET_EXPORT Q_DECL_EXPORT
#else
#define AIWIDGET_EXPORT Q_DECL_IMPORT
#endif
#else
#define AIWIDGET_EXPORT
#endif

#endif // AIWIDGET_AIWIDGET_GLOBAL_H
