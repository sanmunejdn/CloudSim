#ifndef CLOUDSIMPLUGINHOST_AISCENESNAPSHOTBUILDER_H
#define CLOUDSIMPLUGINHOST_AISCENESNAPSHOTBUILDER_H

/// @file AiSceneSnapshotBuilder.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 活动文档对象快照，供 Agent / 确认面板下拉

#include <QByteArray>

class PluginHostContext;

namespace AiSceneSnapshotBuilder
{
QByteArray buildJson(PluginHostContext& host);
}

#endif
