#ifndef CLOUDSIMPLUGINHOST_AIACTIONPLANEXECUTOR_H
#define CLOUDSIMPLUGINHOST_AIACTIONPLANEXECUTOR_H

/// @file AiActionPlanExecutor.h
/// @brief AiActionPlanExecutor 接口

#include <QByteArray>
#include <QString>

class PluginHostContext;

namespace AiActionPlanExecutor
{
bool execute(const PluginHostContext& host, const QByteArray& planJsonUtf8, QString* outSummary, QString* outError);
}

#endif // CLOUDSIMPLUGINHOST_AIACTIONPLANEXECUTOR_H
