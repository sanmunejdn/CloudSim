#pragma once

#include <QByteArray>
#include <QString>

class PluginHostContext;

namespace AiActionPlanExecutor
{
bool execute(const PluginHostContext& host, const QByteArray& planJsonUtf8, QString* outSummary, QString* outError);
}
