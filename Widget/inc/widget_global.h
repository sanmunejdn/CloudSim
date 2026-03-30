#ifndef _POINTCLOUDPROCESS_WIDGET_GLOBAL_H_
#define _POINTCLOUDPROCESS_WIDGET_GLOBAL_H_

// Widget ???????????????? execution_character_set("utf-8") ??????????

#include <QtCore/qglobal.h>

#if defined(_WIN64) || defined(_WIN32)
#pragma execution_character_set("utf-8")
#endif

#ifndef BUILD_STATIC
# if defined(WIDGET_LIB)
#  define WIDGET_EXPORT Q_DECL_EXPORT
# else
#  define WIDGET_EXPORT Q_DECL_IMPORT
# endif
#else
# define WIDGET_EXPORT
#endif

#define HPL_TRY \
	try{

#define HPL_CATCH \
	}catch (const std::exception& exc){ \
		qCritical() << exc.what(); \
	}catch (...){ \
		qCritical() << "��ָ�롢Ұָ�룺" << __FILE__ << ", " << __func__ << ", " << __LINE__;}

#endif //_POINTCLOUDPROCESS_WIDGET_GLOBAL_H_