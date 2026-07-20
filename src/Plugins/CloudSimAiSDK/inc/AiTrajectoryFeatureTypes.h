#ifndef CLOUDSIMAISDK_AITRAJECTORYFEATURETYPES_H
#define CLOUDSIMAISDK_AITRAJECTORYFEATURETYPES_H

/// @file AiTrajectoryFeatureTypes.h
/// @brief 轨迹生成页当前 STEP 工件上下文

#include "cloudsim_ai_sdk_global.h"

#include <QString>

/// 轨迹生成页当前 STEP 工件上下文
struct CLOUDSIM_AI_SDK_EXPORT AiTrajectoryWorkpieceContext
{
	QString backendId;
	QString stepPathUtf8;
	bool valid = false;
};

/// 线/面特征轴（trajectory.feature 意图消歧）
enum class CLOUDSIM_AI_SDK_EXPORT AiFeatureAxis
{
	Ambiguous,
	Line,
	Surface
};

inline QString aiFeatureAxisToString(AiFeatureAxis axis)
{
	switch (axis)
	{
	case AiFeatureAxis::Line:
		return QStringLiteral("line");
	case AiFeatureAxis::Surface:
		return QStringLiteral("surface");
	default:
		return QStringLiteral("ambiguous");
	}
}

#endif // CLOUDSIMAISDK_AITRAJECTORYFEATURETYPES_H
