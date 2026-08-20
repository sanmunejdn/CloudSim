#ifndef PLCCOMMUI_PLC_COMM_UI_GLOBAL_H
#define PLCCOMMUI_PLC_COMM_UI_GLOBAL_H

/// @file plc_comm_ui_global.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PlcCommUI 导出宏

#include <QtCore/qglobal.h>

#if defined(PLCCOMM_UI_STATIC) || defined(BUILD_STATIC)
#define PLCCOMM_UI_EXPORT
#elif defined(PLCCOMM_UI_LIB)
#define PLCCOMM_UI_EXPORT Q_DECL_EXPORT
#else
#define PLCCOMM_UI_EXPORT Q_DECL_IMPORT
#endif

#endif // PLCCOMMUI_PLC_COMM_UI_GLOBAL_H
