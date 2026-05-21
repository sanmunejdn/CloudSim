#ifndef AIWIDGET_GLOBAL_H
#define AIWIDGET_GLOBAL_H

#include <QtCore/qglobal.h>

#if defined(BUILD_STATIC)
# define AIWIDGET_EXPORT
#elif defined(_WIN32) || defined(_WIN64)
# if defined(AIWIDGET_LIB)
#  define AIWIDGET_EXPORT Q_DECL_EXPORT
# else
#  define AIWIDGET_EXPORT Q_DECL_IMPORT
# endif
#else
# define AIWIDGET_EXPORT
#endif

#endif
