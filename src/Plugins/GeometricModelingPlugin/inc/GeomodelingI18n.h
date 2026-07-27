#ifndef GEOMETRICMODELINGPLUGIN_GEOMODDELINGI18N_H
#define GEOMETRICMODELINGPLUGIN_GEOMODDELINGI18N_H

/// @file GeomodelingI18n.h
/// @brief 跟随宿主 useChinese()；默认中文

#include <QString>

inline QString gmTr(bool useChinese, const QString& en, const QString& zh)
{
	return useChinese ? zh : en;
}

#endif
