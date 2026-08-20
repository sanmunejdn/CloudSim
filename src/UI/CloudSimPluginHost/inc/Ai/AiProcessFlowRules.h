#ifndef CLOUDSIMPLUGINHOST_AIPROCESSFLOWRULES_H
#define CLOUDSIMPLUGINHOST_AIPROCESSFLOWRULES_H

/// @file AiProcessFlowRules.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief process.flow 短口语 → 有序 Plan（模板图 + 仿真）

#include "AiAgentTypes.h"

#include <QString>

namespace AiProcessFlowRules
{
AiAgentPlan tryBuildPlan(const QString& userText);
} // namespace AiProcessFlowRules

#endif
