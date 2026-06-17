#pragma once

#include "cloudsim_ai_sdk_global.h"

#include <QString>
#include <QStringList>

/// 领域输出：通用步骤计划或领域自定义 JSON
enum class CLOUDSIM_AI_SDK_EXPORT AiDomainOutputKind
{
	ActionPlan,
	StructuredJson
};

/// 路由/UI 常用领域 id（稳定字符串，勿改已发布值）
namespace AiDomainIds
{
inline QString meshCreate() { return QStringLiteral("mesh.create"); }
inline QString meshCompose() { return QStringLiteral("mesh.compose"); }
inline QString documentImport() { return QStringLiteral("document.import"); }
inline QString pointCloudOps() { return QStringLiteral("pointcloud.ops"); }
inline QString geometryRecognize() { return QStringLiteral("geometry.recognize"); }
inline QString trajectoryFeature() { return QStringLiteral("trajectory.feature"); }
inline QString robotCommand() { return QStringLiteral("robot.command"); }
inline QString pointNetClassify() { return QStringLiteral("pointnet.classify"); }
inline QString pointNetSegment() { return QStringLiteral("pointnet.segment"); }
inline QString autoDomain() { return QStringLiteral("auto"); }
}

struct CLOUDSIM_AI_SDK_EXPORT AiDomainDescriptor
{
	QString domainId;
	QString displayName;
	AiDomainOutputKind outputKind = AiDomainOutputKind::ActionPlan;
	QByteArray catalogSliceJson;
	bool supportsMultimodal = false;
	QStringList parserPriority;
	bool unloadOtherModelsBeforeInfer = false;
};
