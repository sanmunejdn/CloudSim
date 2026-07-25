#ifndef CLOUDSIMPLUGINHOST_AIHOSTBUTTONAPIDISPATCH_H
#define CLOUDSIMPLUGINHOST_AIHOSTBUTTONAPIDISPATCH_H

/// @file AiHostButtonApiDispatch.h
/// @brief Dock 按钮对应 Host API 的 ActionPlan 分发（含异步等待）

#include "AiAgentTypes.h"

#include <QString>

#include <json.hpp>

class PluginHostContext;

namespace AiHostButtonApiDispatch
{
/// handled=false 表示未识别该 api
AiToolResult execute(PluginHostContext& host, const std::string& api, const nlohmann::json& args,
					 bool allowModalDialogs = true);

/// 兼容旧调用：返回是否已处理；失败时写 outError；成功摘要可写 outSummary
bool tryExecute(PluginHostContext& host, const std::string& api, const nlohmann::json& args, QString* outError,
				bool allowModalDialogs = true, QString* outSummary = nullptr);
} // namespace AiHostButtonApiDispatch

#endif
