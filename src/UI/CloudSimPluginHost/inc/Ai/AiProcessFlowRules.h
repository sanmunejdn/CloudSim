#ifndef CLOUDSIMPLUGINHOST_AIPROCESSFLOWRULES_H
#define CLOUDSIMPLUGINHOST_AIPROCESSFLOWRULES_H

/// @file AiProcessFlowRules.h
/// @brief process.flow 短口语 → 有序 Plan（模板图 + 仿真）

#include "AiAgentTypes.h"

#include <QString>

namespace AiProcessFlowRules
{
AiAgentPlan tryBuildPlan(const QString& userText);
} // namespace AiProcessFlowRules

#endif
