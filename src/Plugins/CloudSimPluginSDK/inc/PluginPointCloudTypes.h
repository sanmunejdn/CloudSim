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

enum class PluginPointCloudTemplateBrepRegistrationStage : int
{
	Full = 0,
	CoarseOnly = 1,
	FineOnly = 2,
};

struct PluginPointCloudTemplateBrepUpdateParams
{
	std::string templateBrepBackendIdUtf8;
	PluginPointCloudTemplateBrepRegistrationStage registrationStage =
		PluginPointCloudTemplateBrepRegistrationStage::Full;
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

enum class PluginMeshSurfaceNurbsFitMode : int
{
	Interpolate = 1,
	ApproxFixedCtrlpts = 2,
	ApproxCentripetal = 3,
	ApproxCentripetalFixedCtrlpts = 4,
};

struct PluginMeshSurfaceReconstructParams
{
	int normalSmoothIterations = 6;
	double featureThresholdC0 = 0.8;
	bool runVcgRepairFirst = true;
	bool runIsotropicRemesh = true;
	double remeshTargetEdgeLengthMm = 0.0;
	int remeshIterations = 3;
	double remeshFeatureAngleDeg = 0.0;
	int patchCountHint = 0;
	/// 分块前法向平滑迭代，0=保留锐角
	int partitionNormalSmoothIters = 2;
	/// 特征棱角度百分位 [0.5,0.99]
	double featureAnglePercentile = 0.88;
	int samplesPerPatchEdge = 16;
	double targetUvSpacingMm = 0.0;
	int minSamplesPerEdge = 4;
	int maxSamplesPerEdge = 0;
	int maxFitGridPerEdge = 9;
	double fitUvSpacingMm = 0.0;
	double sampleRateFactor = 2.0;
	int sampleGridMin = 10;
	int sampleGridMax = 2000;
	double controlPointDensityFactor = 0.5;
	int minControlPointsPerDirection = 6;
	int nurbsDegreeU = 3;
	int nurbsDegreeV = 3;
	PluginMeshSurfaceNurbsFitMode fitMode = PluginMeshSurfaceNurbsFitMode::ApproxFixedCtrlpts;
	int parameterGridMode = 1;
	double fitEvaluationDelta = 0.05;
	double blendStripWidth = 0.0;
	double fairingEpsilon = 1e-3;
	int fairingMaxIterations = 50;
	double tessellateLinearDeflectionMm = 0.1;
	QString displayName;
	bool selectInTree = true;
	bool exportPreprocessedMeshToScene = true;
};

enum class PluginMeshSurfaceReconstructStage : int
{
	None = 0,
	Preprocess = 1,
	Partition = 2,
	Sample = 3,
	Fit = 4,
	BoundaryBlend = 5,
	JunctionBlend = 6,
	Fair = 7,
	Assemble = 8,
};

struct PluginMeshSurfaceReconstructSessionId
{
	std::string value;
	bool valid() const { return !value.empty(); }
};

struct PluginMeshSurfaceReconstructReport
{
	PluginMeshSurfaceReconstructStage lastCompletedStage = PluginMeshSurfaceReconstructStage::None;
	QString stageSummaryZh;
	int patchCount = 0;
	int junctionCount = 0;
	double maxDeviationMm = 0.0;
	double globalFairingMetric = 0.0;
	double normalSmoothGapVolume = 0.0;
	bool c2BlendSucceeded = false;
	std::string newBrepBackendId;
	std::string preprocessedMeshBackendId;
	std::string partitionColoredMeshBackendId;
	std::string fitPreviewBrepBackendId;

	int inputTriangleCount = 0;
	int repairedTriangleCount = 0;
	int remeshedTriangleCount = 0;
	double remeshTargetEdgeLengthUsedMm = 0.0;
	int totalSamplePoints = 0;
	int bsplinePatchCount = 0;
	int nurbsPatchCount = 0;
	int planeFallbackCount = 0;
	int amrtoHarmonicSampleCount = 0;
	int outputFaceCount = 0;
	double avgFacesPerPatch = 0.0;
	int minFacesPerPatch = 0;
	int maxFacesPerPatch = 0;
	int smallPatchCount = 0;
	int gridN = 0;
	int gridNuMax = 0;
	int gridNvMax = 0;

	int fitRejectApprox = 0;
	int fitRejectPole = 0;
	int fitRejectFitGrid = 0;
	int fitRejectFullGrid = 0;
	int fitRejectMakeFace = 0;
};

using PluginMeshSurfaceReconstructFinishedFn = std::function<void(
	bool ok,
	const QString& error,
	const PluginMeshSurfaceReconstructReport& report)>;
