#pragma once

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
	float colorRgb[3] = { 0.5f, 0.5f, 0.5f };
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

struct TrainingJobConfig
{
	std::string datasetRootUtf8;
	std::string outputDirUtf8;
	int numClasses = 4;
	int numPoints = 2048;
	int epochs = 100;
	int batchSize = 16;
	double learningRate = 0.001;
};

struct TrainingEpochMetrics
{
	int epoch = 0;
	int totalEpochs = 0;
	double trainLoss = 0.0;
	double trainAcc = 0.0;
	double valLoss = 0.0;
	double valAcc = 0.0;
	double lr = 0.0;
	double elapsedS = 0.0;
	bool best = false;
};

struct TrainingJobResult
{
	bool ok = false;
	std::string statusUtf8;
	double bestValAcc = 0.0;
	std::string bestCheckpointUtf8;
	std::string deviceUtf8;
};
