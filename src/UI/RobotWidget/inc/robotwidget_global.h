#ifndef ROBOTWIDGET_GLOBAL_H
#define ROBOTWIDGET_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
# define ROBOTWIDGET_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
# if defined(ROBOTWIDGET_LIB)
#  define ROBOTWIDGET_EXPORT Q_DECL_EXPORT
# else
#  define ROBOTWIDGET_EXPORT Q_DECL_IMPORT
# endif
#else
# define ROBOTWIDGET_EXPORT
#endif

#endif
