#ifndef CLOUDSIMAISDK_AIINFERENCETYPES_H
#define CLOUDSIMAISDK_AIINFERENCETYPES_H

/// @file AiInferenceTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief AiInferenceTypes 接口

#include "cloudsim_ai_sdk_global.h"

#include <QByteArray>
#include <QString>
#include <functional>

enum class CLOUDSIM_AI_SDK_EXPORT AiInferenceKind
{
	RulesOnly,
	LocalText,
	LocalMultimodal,
	RemoteCloud
};

struct AiInferenceRequest
{
	QString domainId;
	QString userText;
	QByteArray imagePng;
	/// 轨迹页 STEP 工件 backend id（Coordinator 填充）
	QString workpieceBackendId;
	QString workpieceStepPathUtf8;
	/// 按 featureAxis 切片后的 catalog JSON（供 LLM / rules grounding）
	QByteArray catalogSliceUtf8;
	/// 全量 catalog JSON（rules 回退与会话缓存）
	QByteArray catalogFullUtf8;
};

using AiInferenceProgressFn = std::function<void(double fraction, const QString& message)>;

#endif // CLOUDSIMAISDK_AIINFERENCETYPES_H
