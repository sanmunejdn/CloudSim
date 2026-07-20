#ifndef POINTNETPLUGIN_POINTNETTYPES_H
#define POINTNETPLUGIN_POINTNETTYPES_H

/// @file PointNetTypes.h
/// @brief 分类推理结果

#include <QString>
#include <QStringList>
#include <vector>

/// 分类推理结果
struct PointNetClassifyResult
{
	int classId = -1;
	QString className;
	float confidence = 0.0f;
	std::vector<float> probabilities;
};

/// 分割推理结果
struct PointNetSegmentResult
{
	std::vector<int> labels;
	std::vector<float> scores;
	int numClasses = 0;
};

/// 模型配置
struct PointNetModelConfig
{
	QString path;
	int numPoints = 1024;
	QStringList classes;
	int numClasses = 0;
};

/// 插件全局配置
struct PointNetPluginConfig
{
	PointNetModelConfig classifyModel;
	PointNetModelConfig segmentModel;
	QString inferenceProvider = QStringLiteral("cpu");
};

#endif // POINTNETPLUGIN_POINTNETTYPES_H
