#pragma once

#include <QtCore/qglobal.h>

#if defined(PLCCOMM_UI_STATIC) || defined(BUILD_STATIC)
# define PLCCOMM_UI_EXPORT
#elif defined(PLCCOMM_UI_LIB)
# define PLCCOMM_UI_EXPORT Q_DECL_EXPORT
#else
# define PLCCOMM_UI_EXPORT Q_DECL_IMPORT
#endif
