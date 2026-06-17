#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

class IPluginDocument;
class QString;

enum class PluginLabelingGeometryKind
{
	PointCloud,
	TriangleMesh
};

enum class PluginLabelingTool
{
	Click,
	Brush,
	Polyline,
	Erase
};

struct PluginLabelingClassDef
{
	int classId = 0;
	std::string nameUtf8;
	float colorRgb[3] = { 0.5f, 0.5f, 0.5f };
};

struct PluginLabelingSessionConfig
{
	std::vector<PluginLabelingClassDef> classes;
	int unlabeledClassId = 0;
	float defaultBrushRadiusPx = 16.f;
};

struct PluginLabelingSelectionResult
{
	std::vector<std::size_t> pointIndices;
	std::vector<int> triangleIndices;
};

struct PluginLabelingSessionSummary
{
	PluginLabelingGeometryKind geometryKind = PluginLabelingGeometryKind::PointCloud;
	std::size_t totalElements = 0U;
	std::size_t labeledElements = 0U;
	int activeClassId = 0;
	std::map<int, std::size_t> classHistogram;
};

struct PluginLabelingDatasetExportOptions
{
	std::string sampleNameUtf8;
	int numClasses = 0;
	/// 网格导出时表面采样点数
	int meshSampleCount = 2048;
};

struct PluginLabelingDatasetExportResult
{
	bool ok = false;
	std::string plyRelativePath;
	std::string labelRelativePath;
	std::string datasetJsonlPath;
};

using PluginLabelingSessionId = std::uint64_t;
constexpr PluginLabelingSessionId kInvalidLabelingSessionId = 0U;

using PluginLabelingPickFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginLabelingSelectionResult& result)>;

using PluginLabelingBrushStrokeFn = std::function<void(const PluginLabelingSelectionResult& stroke)>;

using PluginLabelingBrushFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginLabelingSelectionResult& total)>;
