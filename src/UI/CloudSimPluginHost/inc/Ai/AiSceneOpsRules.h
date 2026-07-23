#ifndef CLOUDSIMPLUGINHOST_AISCENEOPSRULES_H
#define CLOUDSIMPLUGINHOST_AISCENEOPSRULES_H

/// @file AiSceneOpsRules.h
/// @brief scene.ops 多段口语 → 有序 Plan

#include "AiAgentTypes.h"

#include <QByteArray>
#include <QString>

namespace AiSceneOpsRules
{
/// 解析删除/平移/旋转多段；无法识别返回空 steps
AiAgentPlan tryBuildPlan(const QString& userText, const QByteArray& sceneSnapshotUtf8);
} // namespace AiSceneOpsRules

#endif
