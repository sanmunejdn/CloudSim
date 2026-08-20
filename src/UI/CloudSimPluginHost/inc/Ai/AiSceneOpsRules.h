#ifndef CLOUDSIMPLUGINHOST_AISCENEOPSRULES_H
#define CLOUDSIMPLUGINHOST_AISCENEOPSRULES_H

/// @file AiSceneOpsRules.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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
