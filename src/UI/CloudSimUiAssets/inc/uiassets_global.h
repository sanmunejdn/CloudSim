#pragma once

#include <QtCore/qglobal.h>

#if defined(_WIN64) || defined(_WIN32)
#pragma execution_character_set("utf-8")
#endif

#ifndef BUILD_STATIC
# if defined(UIASSETS_LIB)
#  define UIASSETS_EXPORT Q_DECL_EXPORT
# else
#  define UIASSETS_EXPORT Q_DECL_IMPORT
# endif
#else
# define UIASSETS_EXPORT
#endif
