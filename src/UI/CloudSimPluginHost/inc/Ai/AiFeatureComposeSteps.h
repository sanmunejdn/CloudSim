#ifndef CLOUDSIMPLUGINHOST_AIFEATURECOMPOSESTEPS_H
#define CLOUDSIMPLUGINHOST_AIFEATURECOMPOSESTEPS_H

/// @file AiFeatureComposeSteps.h
/// @brief feature.compose 步骤 → Parametric Host

#include <QHash>
#include <QString>

#include <json.hpp>
#include <string>

class PluginHostContext;

namespace AiFeatureComposeSteps
{
/// @return false=非本域 API；true=已处理，成功与否看 outError 是否为空
bool tryExecute(PluginHostContext& host, const std::string& api, const nlohmann::json& args, const std::string& stepId,
				QHash<QString, QString>& stepIdToBackendId, QString* outError);
}

#endif
