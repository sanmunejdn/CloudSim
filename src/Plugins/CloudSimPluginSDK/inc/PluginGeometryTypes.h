#ifndef CLOUDSIMPLUGINSDK_PLUGINGEOMETRYTYPES_H
#define CLOUDSIMPLUGINSDK_PLUGINGEOMETRYTYPES_H

/// @file PluginGeometryTypes.h
/// @brief STEP 内拓扑元素引用（edge/face 索引从 0 起）

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
	/// 1.24+：内存 B-rep / ParametricBody 无 STEP 时用 backendId 定位
	std::string backendIdUtf8;
	/// 1.40.0+：视口命中点（世界 mm）；Vertex 拾取时为近端点
	PluginPoint3d hitWorldMm{};
	bool hasHitPoint = false;
};

enum class PluginGeometryElementKind
{
	Edge = 0,
	Face,
	/// 1.40.0+：边拾取后吸附到靠近点击的端点
	Vertex
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

using PluginGeometryFinishedFn =
	std::function<void(bool ok, const QString& error, const PluginGeometryJobResult& result)>;

using PluginGeometryElementPickedFn =
	std::function<void(bool ok, const QString& error, const PluginGeometryStepRef& ref)>;

/// 草图平面（世界 mm）
struct PluginSketchPlane
{
	PluginPoint3d origin{};
	PluginPoint3d axisX{};
	PluginPoint3d axisY{};
	PluginPoint3d normal{};
	bool isPlanar = false;
};

enum class PluginSketchExtrudeMode
{
	Pad = 0,
	Pocket
};

enum class PluginSketchExtrudeEnd
{
	Blind = 0,
	UpToFace,
	MidPlane,
	ThroughAll,
	UpToVertex,
	OffsetFromFace
};

struct PluginSketchExtrudeParams
{
	PluginSketchExtrudeMode mode = PluginSketchExtrudeMode::Pad;
	double lengthMm = 10.0;
	bool reversed = false;
	PluginSketchExtrudeEnd endCondition = PluginSketchExtrudeEnd::Blind;
	PluginSketchPlane upToFacePlane{};
	bool hasUpToFacePlane = false;
	/// 兼容旧路径：Pocket 时基实体；参数化 Body 模式下忽略（tip 在 Body 内）
	std::string baseBackendIdUtf8;
	std::string resultNameUtf8;
	/// 1.23.0+：空=新建 ParametricBrepModel；非空=追加到该 Body
	std::string targetParametricBackendIdUtf8;
	/// 1.27.0+：可选完整草图文档 JSON，写入对应 Sketch 特征
	std::string sketchDocumentJsonUtf8;
	/// 1.29.0+：UpToFace 弱拓扑引用（rebuild 重解，失败回退烤平面）
	std::string upToFaceBackendIdUtf8;
	int upToFaceIndex = -1;
	/// 1.32.0+：拔模斜度（度），默认 0
	double draftAngleDeg = 0.0;
	/// 1.39.0+：UpToVertex 目标点（世界 mm）
	PluginPoint3d upToVertex{};
	bool hasUpToVertex = false;
	/// 1.39.0+：OffsetFromFace 沿拉伸方向偏移
	double offsetFromFaceMm = 0.0;
	/// 1.38.0+：内孔环（闭合折线 xyz）
	std::vector<std::vector<float>> holePolylinesXyzMm;
};

enum class PluginSketchSweepMode
{
	Boss = 0,
	Cut
};

enum class PluginSketchSweepPathSegKind
{
	Line = 0,
	Arc,
	SplineThrough
};

struct PluginSketchSweepPathSegment
{
	PluginSketchSweepPathSegKind kind = PluginSketchSweepPathSegKind::Line;
	float ax = 0.f;
	float ay = 0.f;
	float az = 0.f;
	float bx = 0.f;
	float by = 0.f;
	float bz = 0.f;
	float mx = 0.f;
	float my = 0.f;
	float mz = 0.f;
};

struct PluginSketchSweepParams
{
	PluginSketchSweepMode mode = PluginSketchSweepMode::Boss;
	std::string resultNameUtf8;
	/// 空=新建 Body（仅 Boss）；Cut 必须非空
	std::string targetParametricBackendIdUtf8;
	/// 已有 Sketch id；空则 Host 新建
	std::string profileSketchIdUtf8;
	std::string pathSketchIdUtf8;
	std::string profileSketchDocumentJsonUtf8;
	std::string pathSketchDocumentJsonUtf8;
	PluginSketchPlane profilePlane{};
	PluginSketchPlane pathPlane{};
	/// 非空则优先真弧/线段建 wire；空则回退 path 折线
	std::vector<PluginSketchSweepPathSegment> pathSegments;
	/// 1.39.0+：截面绕路径起点切向扭转（度）
	double twistDeg = 0.0;
};

struct PluginSketchFilletParams
{
	double radiusMm = 1.0;
	std::vector<int> edgeIndices;
	std::string targetParametricBackendIdUtf8;
	std::string resultNameUtf8;
	/// 1.43.0+：为 true 时圆角 tip 全部边（忽略 edgeIndices）
	bool allEdges = false;
};

struct PluginSketchChamferParams
{
	double distanceMm = 1.0;
	std::vector<int> edgeIndices;
	std::string targetParametricBackendIdUtf8;
	std::string resultNameUtf8;
};

enum class PluginSketchRevolveMode
{
	Boss = 0,
	Cut
};

struct PluginSketchRevolveParams
{
	PluginSketchRevolveMode mode = PluginSketchRevolveMode::Boss;
	double angleDeg = 360;
	double axisOx = 0;
	double axisOy = 0;
	double axisOz = 0;
	double axisDx = 0;
	double axisDy = 0;
	double axisDz = 1;
	std::string targetParametricBackendIdUtf8;
	std::string sketchIdUtf8;
	std::string sketchDocumentJsonUtf8;
	PluginSketchPlane plane{};
	std::string resultNameUtf8;
};

struct PluginSketchLinearPatternParams
{
	int count = 2;
	double dxMm = 10;
	double dyMm = 0;
	double dzMm = 0;
	/// 1.41.0+：上游特征 id；空=阵列当前 tip
	std::string sourceFeatureIdUtf8;
	std::string targetParametricBackendIdUtf8;
	std::string resultNameUtf8;
};

struct PluginSketchMirror3dParams
{
	PluginSketchPlane plane{};
	bool keepOriginal = true;
	std::string targetParametricBackendIdUtf8;
	std::string resultNameUtf8;
};

enum class PluginSketchLoftMode
{
	Boss = 0,
	Cut
};

struct PluginSketchLoftParams
{
	PluginSketchLoftMode mode = PluginSketchLoftMode::Boss;
	std::string targetParametricBackendIdUtf8;
	std::string sketchAIdUtf8;
	std::string sketchBIdUtf8;
	std::string sketchADocumentJsonUtf8;
	std::string sketchBDocumentJsonUtf8;
	PluginSketchPlane planeA{};
	PluginSketchPlane planeB{};
	std::string resultNameUtf8;
};

struct PluginSketchShellParams
{
	double thicknessMm = 1.0;
	std::vector<int> faceIndices;
	std::string targetParametricBackendIdUtf8;
	std::string resultNameUtf8;
};

struct PluginSketchDraftParams
{
	double angleDeg = 1.0;
	std::vector<int> faceIndices;
	PluginSketchPlane neutralPlane{};
	std::string targetParametricBackendIdUtf8;
	std::string resultNameUtf8;
};

struct PluginSketchOverlaySegment
{
	std::vector<float> xyzMm; ///< 折线或圆离散点
	bool construction = false;
	/// 1.27.0+：诊断着色（默认青）
	float rgba[4] = {0.20f, 0.85f, 1.00f, 1.00f};
	/// 1.27.0+：线宽像素，0=宿主默认
	float lineWidthPx = 0.0f;
};

enum class PluginSketchInputKind
{
	MouseMove = 0,
	MousePress,
	MouseRelease,
	KeyPress
};

struct PluginSketchInputEvent
{
	PluginSketchInputKind kind = PluginSketchInputKind::MouseMove;
	int screenX = 0;
	int screenY = 0;
	/// Qt::LeftButton=1, RightButton=2, MiddleButton=4；键盘时为 Qt::Key
	int buttonOrKey = 0;
	int modifiers = 0;
	bool hasWorldHit = false;
	PluginPoint3d worldMm{};
};

/// 返回 true：已消费（抑制视口轨道）
using PluginSketchInputFn = std::function<bool(const PluginSketchInputEvent& ev)>;

enum class PluginOriginPlaneKind
{
	XY = 0,
	XZ = 1,
	YZ = 2
};

using PluginOriginPlanePickedFn =
	std::function<void(bool ok, const QString& error, PluginOriginPlaneKind kind, const PluginSketchPlane& plane)>;

/// 1.33.0+：单视图 HLR 折线（xy 交错，无 z）
struct PluginDrawingHlrViewResult
{
	std::string viewId;
	std::vector<std::vector<float>> visibleXy;
	std::vector<std::vector<float>> hiddenXy;
};

struct PluginDrawingHlrResult
{
	std::vector<PluginDrawingHlrViewResult> views;
};

using PluginDrawingHlrFinishedFn =
	std::function<void(bool ok, const QString& error, const PluginDrawingHlrResult& result)>;

/// 1.34.0+：工程图投影参数；1.42.0+ 追加自定义剖切平面
struct PluginDrawingProjectParams
{
	bool thirdAngle = false;
	bool includeIso = true;
	bool includeSection = false;
	/// 0=正视平行中面 1=俯视平行 2=右视平行（customSection=false 时）
	int sectionPlane = 0;
	/// 1.42.0+：为 true 时用 sectionOriginMm / sectionNormal
	bool customSection = false;
	double sectionOriginMm[3] = {0.0, 0.0, 0.0};
	double sectionNormal[3] = {0.0, 1.0, 0.0};
};

/// 1.37.0+：世界原点 + XY/XZ/YZ 基准面显隐（建模特征树眼开关）
struct PluginOriginReferenceVisibility
{
	bool originPoint = true;
	bool planeXY = true;
	bool planeXZ = true;
	bool planeYZ = true;
};

#endif // CLOUDSIMPLUGINSDK_PLUGINGEOMETRYTYPES_H
