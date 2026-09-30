#ifndef WIDGET_OSGSCENENODEI18N_H
#define WIDGET_OSGSCENENODEI18N_H

/// @file OsgSceneNodeI18n.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 场景层级 OSG 节点名/类型中文映射

#include "widget_global.h"

#include <QString>

namespace OsgSceneNodeI18n
{
WIDGET_EXPORT QString translateClassName(const QString& cls, bool useChinese);
WIDGET_EXPORT QString translateNodeName(const QString& name, bool useChinese);
} // namespace OsgSceneNodeI18n

#endif
