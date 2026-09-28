#ifndef CLOUDSIMLABELINGSDK_LABELINGTYPES_H
#define CLOUDSIMLABELINGSDK_LABELINGTYPES_H

/// @file LabelingTypes.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief LabelingTypes 接口

#include "labeling_sdk_global.h"

#include <cstddef>
#include <map>
#include <string>
#include <vector>

enum class LabelingGeometryKind
{
	PointCloud,
	TriangleMesh
};

struct LabelingClassDef
{
	int classId = 0;
	std::string nameUtf8;
	float colorRgb[3] = {0.5f, 0.5f, 0.5f};
};

struct LabelingSessionConfig
{
	std::vector<LabelingClassDef> classes;
	int unlabeledClassId = 0;
};

struct LabelingUndoPatch
{
	std::vector<std::size_t> indices;
	std::vector<int> previousLabels;
};

struct LabelingDatasetExportOptions
{
	std::string sampleNameUtf8;
	int numClasses = 0;
	int meshSampleCount = 2048;
};

struct LabelingDatasetExportResult
{
	bool ok = false;
	std::string plyRelativePath;
	std::string labelRelativePath;
	std::string datasetJsonlPath;
};

// 训练 POD 真源在 PluginSDK PluginLabelingTypes.h（插件 UI）；本 SDK 仅为宿主 LabelingSession

#endif // CLOUDSIMLABELINGSDK_LABELINGTYPES_H
