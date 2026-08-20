#ifndef CLOUDSIMPLUGINHOST_AIACTIONPLANEXECUTOR_H
#define CLOUDSIMPLUGINHOST_AIACTIONPLANEXECUTOR_H

/// @file AiActionPlanExecutor.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AiActionPlanExecutor 接口

#include <QByteArray>
#include <QString>

class PluginHostContext;

namespace AiActionPlanExecutor
{
bool execute(const PluginHostContext& host, const QByteArray& planJsonUtf8, QString* outSummary, QString* outError);

/// create_mesh / Agent 确认前：预览草图+拉伸等步骤文案（非参数化基本体返回空）
QString previewCreateMeshFeatureSteps(const QByteArray& createMeshOrArgsJsonUtf8);
}

#endif // CLOUDSIMPLUGINHOST_AIACTIONPLANEXECUTOR_H
