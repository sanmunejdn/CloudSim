#ifndef GEOMETRICMODELINGPLUGIN_GEOMODDELINGI18N_H
#define GEOMETRICMODELINGPLUGIN_GEOMODDELINGI18N_H

/// @file GeomodelingI18n.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 跟随宿主 useChinese()；默认中文

#include <QString>

inline QString gmTr(bool useChinese, const QString& en, const QString& zh)
{
	return useChinese ? zh : en;
}

#endif
