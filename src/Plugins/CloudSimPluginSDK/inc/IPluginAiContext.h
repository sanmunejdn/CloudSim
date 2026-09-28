#ifndef CLOUDSIMPLUGINSDK_IPLUGINAICONTEXT_H
#define CLOUDSIMPLUGINSDK_IPLUGINAICONTEXT_H

/// @file IPluginAiContext.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AI 与工艺流程 AI 桥窄接口（IPluginHostContext 按域拆分）

/// vtable 仅末尾追加，勿插入中间以免破坏旧插件 ABI

#include "cloudsim_plugin_sdk_global.h"

#include <QByteArray>
#include <QString>
#include <functional>

class IAiAssistantHost;
class IProcessFlowAiBridge;

/// AI 上下文：AI 助手宿主、轨迹识别/计划、特征史通知、工艺流程 AI 桥
class IPluginAiContext
{
public:
	virtual ~IPluginAiContext() = default;

	/// 接口版本（0x00010000 = 1.0.0）
	virtual unsigned int contextVersion() const = 0;

	/// AI 助手宿主（CloudSimAiSDK）；未链 AiSDK 时可为 null
	virtual IAiAssistantHost* aiAssistantHost() = 0;
	virtual const IAiAssistantHost* aiAssistantHost() const = 0;

	/// 轨迹生成页 combo 当前 STEP 工件
	virtual bool resolveTrajectoryWorkpiece(QString& outBackendId, QString& outStepPath,
											QString* outError = nullptr) = 0;

	/// 枚举 catalog 并按用户文本推断 featureAxis 切片
	virtual bool buildTrajectoryFeatureCatalogSlice(const QString& backendId, const QString& stepPathUtf8,
													const QString& userText, QByteArray& outFullCatalogUtf8,
													QByteArray& outSliceUtf8, QString* outError = nullptr) = 0;

	/// 3D 编号高亮预览（catalog 切片 JSON）
	virtual bool showAiFeatureCandidatePreview(const QByteArray& catalogSliceUtf8, QString* outError = nullptr) = 0;
	virtual void clearAiFeatureCandidatePreview() = 0;

	/// 离散选中特征并注入轨迹编辑 session + 默认工艺流水线
	virtual bool commitAiTrajectoryFeatures(const QByteArray& featurePlanJsonUtf8, QString* outSummary,
											QString* outError = nullptr) = 0;

	/// 补全计划并弹出策略/算子确认框；返回 1=接受 0=取消 2=重新识别
	virtual int proposeAndConfirmTrajectoryPlan(const QByteArray& planInUtf8, QByteArray& planOutUtf8,
												QString* outError = nullptr, bool showRetry = true) = 0;
	virtual bool loadBoundTrajectoryPlanForAi(QByteArray& planOutUtf8, QString* outError = nullptr) = 0;
	virtual bool reviseAiTrajectoryPlan(const QByteArray& planJsonUtf8, QString* outSummary,
										QString* outError = nullptr) = 0;

	/// Parametric Body 特征史变更（AI/Host 写入后；插件可 sync 特征树）
	virtual void onParametricBodyHistoryChanged(
		std::function<void(const QString& documentId, const QString& backendId)> callback) = 0;

	/// 工艺流程 AI 桥接（插件注册；未加载时为 null）
	virtual void setProcessFlowAiBridge(IProcessFlowAiBridge* bridge) = 0;
	virtual IProcessFlowAiBridge* processFlowAiBridge() = 0;
	virtual const IProcessFlowAiBridge* processFlowAiBridge() const = 0;
};

#endif // CLOUDSIMPLUGINSDK_IPLUGINAICONTEXT_H
