#pragma once

#include "cloudsim_plugin_sdk_global.h"

#include "PluginPrimitiveTypes.h"

#include <cstddef>
#include <functional>
#include <string>
#include <vector>

class IPluginDocument;
class QString;

struct PluginAxisAlignedBox
{
	PluginVec3 minMm{};
	PluginVec3 maxMm{};
	bool valid = false;
};

struct PluginPointCloudInfo
{
	std::size_t pointCount = 0U;
	PluginAxisAlignedBox bounds{};
	bool hasPerVertexColors = false;
	bool hasPointNormals = false;
};

struct PluginPointCloudMeasure
{
	PluginVec3 centroidMm{};
	PluginAxisAlignedBox bounds{};
	double averageSpacingMm = 0.0;
};

/// 列主序 4×4 刚体变换（mm）
struct PluginMat4
{
	double v[16] = {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0};
};

struct PluginPointCloudJobResult
{
	std::string newBackendId;
	PluginMat4 icpTransform{};
	double rmseMm = 0.0;
	std::size_t pointCountAfter = 0U;
};

struct PluginPointCloudDownsampleVoxelParams
{
	double voxelSizeMm = 2.0;
	unsigned int minPointsPerCell = 1U;
};

struct PluginPointCloudDownsampleRandomParams
{
	double retainedFraction = 0.5;
};

struct PluginPointCloudCropBoxParams
{
	PluginAxisAlignedBox box{};
};

struct PluginPointCloudCropSphereParams
{
	PluginVec3 centerMm{};
	double radiusMm = 10.0;
};

/// 1.11.0+：屏幕多边形裁剪（封闭折线，Qt 逻辑像素）
struct PluginPointCloudCropPolylineParams
{
	std::vector<float> polylineScreenXy;
	double mvpMatrix[16] = {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0};
	double modelToWorld[16] = {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0};
	int viewportWidth = 0;
	int viewportHeight = 0;
	/// true=保留多边形内部；false=删除内部
	bool keepInside = true;
};

struct PluginPointCloudPolylinePickResult
{
	std::vector<float> polylineScreenXy;
	double mvpMatrix[16] = {
		1.0, 0.0, 0.0, 0.0,
		0.0, 1.0, 0.0, 0.0,
		0.0, 0.0, 1.0, 0.0,
		0.0, 0.0, 0.0, 1.0};
	int viewportWidth = 0;
	int viewportHeight = 0;
};

using PluginPointCloudPolylinePickFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginPointCloudPolylinePickResult& result)>;

struct PluginPointCloudOutlierParams
{
	double removalPercent = 5.0;
	unsigned int kNeighbors = 24U;
};

struct PluginPointCloudNormalsParams
{
	unsigned int kNeighbors = 12U;
	unsigned int jetDegreeFitting = 2U;
};

struct PluginPointCloudPreprocessParams
{
	double voxelPrefilterMm = 1.0;
	double outlierRemovalPercent = 5.0;
};

struct PluginPointCloudIcpParams
{
	std::string targetBackendIdUtf8;
	bool applyTransformToSource = true;
	int maxIterations = 40;
	double convergenceTransMm = 0.01;
	double maxPairDistanceMm = 0.0;
	std::size_t icpMaxPoints = 4000U;
};

struct PluginPointCloudTpsControlParams
{
	std::vector<std::size_t> controlPointIndices;
	std::vector<float> controlDisplacementXyz;
	double regularizationLambda = 1e-6;
};

struct PluginPointCloudTpsFitParams
{
	std::string targetBackendIdUtf8;
	std::vector<std::size_t> correspondenceIndices;
	double regularizationLambda = 1e-6;
	bool createNewPointCloud = false;
	PluginMeshCreateOptions newObjectOptions{};
};

struct PluginPointCloudReconstructPoissonParams
{
	double spacingMm = 0.0;
	double smAngleDeg = 20.0;
	double smRadiusRel = 30.0;
	double smDistanceRel = 0.375;
	PluginMeshCreateOptions meshOptions{};
};

struct PluginPointCloudReconstructPoissonAutoParams
{
	double voxelPrefilterMm = 1.0;
	double outlierRemovalPercent = 5.0;
	PluginMeshCreateOptions meshOptions{};
};

struct PluginPointCloudReconstructScaleSpaceParams
{
	std::size_t smoothIterations = 4U;
	double meshingRadiusMm = 0.0;
	PluginMeshCreateOptions meshOptions{};
};

struct PluginPointCloudTemplateBrepUpdateParams
{
	std::string templateBrepBackendIdUtf8;
	double voxelPrefilterMm = 1.0;
	double faceBandMm = 2.0;
	double normalThresholdDeg = 35.0;
	std::size_t minPointsPerFace = 30U;
	double maxAllowedDeviationMm = 0.5;
	std::string displayNameUtf8;
	/// 0-based 面索引；空表示处理全部面
	std::vector<int> selectedFaceIndices;
	/// 选择性重构时每面扫描归属点数上限
	std::size_t maxAssignPointsPerFace = 800U;
	/// BSpline UV 聚合网格（越大越平滑、越慢）
	int bsplineUvGridCellsU = 24;
	int bsplineUvGridCellsV = 12;
	/// BSpline 极点位移场平滑迭代次数
	int bsplinePoleSmoothPasses = 2;
};

struct PluginPointCloudFaceUpdateReport
{
	int faceIndex = -1;
	std::string surfaceTypeName;
	std::string action;
	double maxDeviationMm = 0.0;
};

struct PluginPointCloudTemplateBrepRegisterResult
{
	double icpRmseMm = 0.0;
};

struct PluginPointCloudTemplateBrepUpdateResult
{
	std::string newBrepBackendId;
	std::size_t updatedFaceCount = 0U;
	std::size_t skippedBadBboxFaceCount = 0U;
	double globalMaxDeviationMm = 0.0;
	bool qualityGatePassed = false;
	std::vector<PluginPointCloudFaceUpdateReport> perFace;
};

using PluginPointCloudTemplateBrepRegisterFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginPointCloudTemplateBrepRegisterResult& result)>;

using PluginPointCloudTemplateBrepUpdateFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginPointCloudTemplateBrepUpdateResult& result)>;

struct PluginPointCloudRigidTransformParams
{
	PluginMat4 transform{};
};

// === 网格后处理类型（vcglib，1.9.0+） ===

struct PluginMeshInfo
{
	std::size_t faceCount = 0U;
	std::size_t vertexCount = 0U;
};

struct PluginMeshSimplifyParams
{
	int targetFaceCount = 0;           // 目标面数，0=保留原面数一半
	double qualityThreshold = 0.3;     // 质量阈值 0-1
	bool preserveBoundary = true;
	bool preserveTopology = true;
	PluginMeshCreateOptions resultOptions{}; // 结果对象创建选项
};

struct PluginMeshSmoothParams
{
	int iterations = 3;                // 迭代次数
	double lambda = 0.2;               // Implicit fairing 强度（仅 implicit 模式）
	bool useImplicitFairing = false;   // false=Laplacian, true=Implicit Fairing
	PluginMeshCreateOptions resultOptions{};
};

struct PluginMeshRepairParams
{
	bool removeDegenerate = true;
	bool removeDuplicate = true;
	bool removeNonManifold = true;
	bool fillHoles = false;
	PluginMeshCreateOptions resultOptions{};
};

struct PluginMeshRemeshParams
{
	double targetEdgeLengthMm = 2.0;
	int iterations = 3;
	PluginMeshCreateOptions resultOptions{};
};

using PluginMeshFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginPointCloudJobResult& result)>;

// === 网格缺陷分析（vcglib，1.10.0+） ===

struct PluginMeshDefectFace
{
	int faceIndex = -1;
	int kind = 0;
	double score = 0.0;
};

struct PluginMeshDefectParams
{
	double sensitivity = 0.08;
	int minClusterFaces = 3;
	bool detectNeedle = true;
	bool detectProtrusion = true;
	bool detectBoundarySpike = true;
};

struct PluginMeshDefectReport
{
	int totalFaces = 0;
	int defectFaceCount = 0;
	double defectAreaRatio = 0.0;
	int needleCount = 0;
	int protrusionCount = 0;
	int boundarySpikeCount = 0;
	std::vector<PluginMeshDefectFace> defects;
};

using PluginMeshDefectFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginMeshDefectReport& report)>;

using PluginPointCloudFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginPointCloudJobResult& result)>;
