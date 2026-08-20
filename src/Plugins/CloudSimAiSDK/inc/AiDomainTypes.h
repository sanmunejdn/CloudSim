#ifndef CLOUDSIMAISDK_AIDOMAINTYPES_H
#define CLOUDSIMAISDK_AIDOMAINTYPES_H

/// @file AiDomainTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 领域输出：通用步骤计划或领域自定义 JSON

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
inline QString meshCreate()
{
	return QStringLiteral("mesh.create");
}
inline QString meshCompose()
{
	return QStringLiteral("mesh.compose");
}
/// 顺序特征链：草图拉伸 / 切除 / 圆角 / 阵列 → Parametric Body
inline QString featureCompose()
{
	return QStringLiteral("feature.compose");
}
/// 标准件库：检索规格 + 填模 → feature.compose（毛坯可换 model_ref）
inline QString designParts()
{
	return QStringLiteral("design.parts");
}
/// 设计计算（公式资产）；与 design.parts 并列，运行时可后续接线
inline QString designCalc()
{
	return QStringLiteral("design.calc");
}
inline QString documentImport()
{
	return QStringLiteral("document.import");
}
inline QString pointCloudOps()
{
	return QStringLiteral("pointcloud.ops");
}
inline QString geometryOps()
{
	return QStringLiteral("geometry.ops");
}
inline QString featureBuild()
{
	return QStringLiteral("feature.build");
}
inline QString labelingAnnot()
{
	return QStringLiteral("labeling.annot");
}
inline QString sceneOps()
{
	return QStringLiteral("scene.ops");
}
inline QString processFlow()
{
	return QStringLiteral("process.flow");
}
inline QString geometryRecognize()
{
	return QStringLiteral("geometry.recognize");
}
inline QString trajectoryFeature()
{
	return QStringLiteral("trajectory.feature");
}
inline QString robotCommand()
{
	return QStringLiteral("robot.command");
}
inline QString pointNetClassify()
{
	return QStringLiteral("pointnet.classify");
}
inline QString pointNetSegment()
{
	return QStringLiteral("pointnet.segment");
}
inline QString autoDomain()
{
	return QStringLiteral("auto");
}
} // namespace AiDomainIds

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

#endif // CLOUDSIMAISDK_AIDOMAINTYPES_H
