#ifndef CLOUDSIMPLUGINHOST_AISCENESNAPSHOTBUILDER_H
#define CLOUDSIMPLUGINHOST_AISCENESNAPSHOTBUILDER_H

/// @file AiSceneSnapshotBuilder.h
/// @brief 活动文档对象快照，供 Agent / 确认面板下拉

#include <QByteArray>

class PluginHostContext;

namespace AiSceneSnapshotBuilder
{
QByteArray buildJson(PluginHostContext& host);
}

#endif
