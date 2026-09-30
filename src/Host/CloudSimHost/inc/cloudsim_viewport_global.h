#ifndef CLOUDSIMHOST_CLOUDSIM_VIEWPORT_GLOBAL_H
#define CLOUDSIMHOST_CLOUDSIM_VIEWPORT_GLOBAL_H

/// @file cloudsim_viewport_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief DesktopViewport DLL 导出宏

#include <QtCore/qglobal.h>

#if defined(CLOUDSIM_VIEWPORT_STATIC)
#define CLOUDSIM_VIEWPORT_EXPORT
#elif defined(CLOUDSIM_VIEWPORT_LIB)
#define CLOUDSIM_VIEWPORT_EXPORT Q_DECL_EXPORT
#else
#define CLOUDSIM_VIEWPORT_EXPORT Q_DECL_IMPORT
#endif

#endif // CLOUDSIMHOST_CLOUDSIM_VIEWPORT_GLOBAL_H
