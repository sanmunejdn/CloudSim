#ifndef CLOUDSIMPLUGINSDK_IPROCESSFLOWAIBRIDGE_H
#define CLOUDSIMPLUGINSDK_IPROCESSFLOWAIBRIDGE_H

/// @file IProcessFlowAiBridge.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 工艺流程 AI 门面：Host/Agent 只经此接口触达插件画布与 DES

#include "cloudsim_plugin_sdk_global.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QString>

/// 由 ProcessFlowPlugin 实现并注册到 Host；未加载插件时指针为空
class IProcessFlowAiBridge
{
public:
	virtual ~IProcessFlowAiBridge() = default;

	/// 无活动文档或进入失败时返回 false
	virtual bool ensureEntered(QString* outError = nullptr) = 0;

	/// 校验并整图替换；成功后可选自动排版
	virtual bool applyFlowJson(const QJsonObject& flow, bool autoLayout = true, QString* outError = nullptr) = 0;

	/// 增量改图：ops 为数组，元素含 op=setNodeProp|addNode|removeNode|connect|disconnect
	virtual bool applyFlowPatch(const QJsonArray& ops, QString* outError = nullptr) = 0;

	/// 同步 DES；config 可含 horizonSec / policy / openGantt；outStats 为 SimStatistics JSON
	virtual bool runSimSync(const QJsonObject& config, QJsonObject* outStats = nullptr,
							QString* outError = nullptr) = 0;

	/// 多策略对比；config.includeTraces 时 outRows 带 operationTrace；否则仅汇总
	virtual bool compareSync(const QJsonObject& config, QJsonArray* outRows = nullptr,
							 QString* outError = nullptr) = 0;

	virtual QJsonObject exportFlowJson() const = 0;
};

#endif
