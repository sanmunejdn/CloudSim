#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include "PluginPrimitiveTypes.h"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

class IPluginDocument;
class QString;

struct PluginPoint3d
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

enum class PluginMeshDiscretizeMode
{
	AdaptiveTriangulation = 0,
	UniformRelative,
	UVStructuredGrid,
	WireTubeMesh,
	WireRibbonMesh
};

enum class PluginMeshQualityPreset
{
	Coarse = 0,
	Medium,
	Fine,
	Custom
};

enum class PluginMeshDensityControl
{
	QualityPreset = 0,
	TargetEdgeLength,
	TargetTriangleCount
};

struct PluginMeshDiscretizeParams
{
	PluginMeshDiscretizeMode mode = PluginMeshDiscretizeMode::AdaptiveTriangulation;
	PluginMeshQualityPreset quality = PluginMeshQualityPreset::Medium;
	PluginMeshDensityControl densityControl = PluginMeshDensityControl::QualityPreset;
	double targetEdgeLengthMm = 0.0;
	std::size_t targetTriangleCount = 0;
	double linearDeflectionMm = 0.01;
	bool linearDeflectionRelative = true;
	double angularDeflectionDeg = 0.5;
	int uvGridCountU = 32;
	int uvGridCountV = 32;
	double tubeRadiusMm = 1.0;
	int tubeSides = 12;
	double ribbonWidthMm = 2.0;
};

/// STEP 内拓扑元素引用（edge/face 索引从 0 起）
struct PluginGeometryStepRef
{
	std::string stepPathUtf8;
	int edgeIndex = -1;
	int faceIndex = -1;
};

enum class PluginGeometryElementKind
{
	Edge = 0,
	Face
};

struct PluginGeometryBackendEntry
{
	std::string backendId;
	std::string displayName;
	std::string className;
	std::string stepPathUtf8;
	bool pickable = false;
};

struct PluginGeometryElementPickRequest
{
	/// 可选：限制拾取后端；为空表示允许当前视图命中的任意后端
	std::string backendIdUtf8;
	/// 可选：显式传入 STEP 路径；为空时由后端 sourcePath 解析
	std::string stepPathUtf8;
	PluginGeometryElementKind kind = PluginGeometryElementKind::Face;
};

struct PluginGeometryIntersectionParams
{
	double toleranceMm = 1e-3;
	bool discretizeCurves = true;
	double curveLinearDeflectionMm = 0.01;
};

enum class PluginBrepBooleanOp
{
	Fuse = 0,
	Common,
	Cut
};

struct PluginGeometryBrepBooleanParams
{
	PluginBrepBooleanOp op = PluginBrepBooleanOp::Fuse;
	PluginMeshDiscretizeParams meshParams{};
	PluginMeshCreateOptions resultOptions{};
};

struct PluginGeometryJobResult
{
	std::string newBackendId;
	std::vector<std::vector<float>> polylines;
	std::vector<PluginPoint3d> intersectionPoints;
	std::size_t triangleCount = 0U;
	double avgEdgeLengthMm = 0.0;
	double maxResidualMm = 0.0;
};

using PluginGeometryFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginGeometryJobResult& result)>;

using PluginGeometryElementPickedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginGeometryStepRef& ref)>;
