#ifndef GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTION_H
#define GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTION_H

/// @file MeshSurfaceReconstruction.h
/// @brief 分块完成后按片着色三角 soup（rgb 与 soup 同布局，每顶点 3 float）

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"

#include <memory>
#include <string>
#include <vector>

namespace geoalgo
{
enum class MeshSurfaceNurbsFitMode : int
{
	Interpolate = 1,
	ApproxFixedCtrlpts = 2,
	ApproxCentripetal = 3,
	ApproxCentripetalFixedCtrlpts = 4,
};

enum class MeshSurfacePartitionMode : int
{
	GeodesicVoronoiV3 = 0,
	HybridNormalCvt = 1,
	/// Hybrid + CGAL SDF 种子融合，面向 TO/棱柱面
	CgalChartHybrid = 2,
	/// AMRTO：Instant-Meshes + GMCG 稀疏四边域
	AmrtoImGmcg = 3,
};

enum class MeshSurfaceGmcgBackend : int
{
	GoldenLoader = 0,
	Exe = 1,
	Native = 2,
};

enum class MeshSurfaceHarmonicBoundaryMode : int
{
	Circular = 0,
	GeodesicSquare = 1,
};

struct MeshSurfaceReconstructParams
{
	int normalSmoothIterations = 6;
	double featureThresholdC0 = 0.8;
	bool runVcgRepairFirst = true;
	bool runIsotropicRemesh = true;
	/// ≤0 时预处理均匀化取修复后网格边长中位数
	double remeshTargetEdgeLengthMm = 0.0;
	int remeshIterations = 3;
	/// ≤0 时用 featureThresholdC0 转角度，仍无效则 30°
	double remeshFeatureAngleDeg = 0.0;

	int patchCountHint = 0;
	MeshSurfacePartitionMode partitionMode = MeshSurfacePartitionMode::GeodesicVoronoiV3;
	/// 分块前法向平滑迭代，稳定特征棱判定
	int partitionNormalSmoothIters = 2;
	/// 特征棱角度百分位 [0.5,0.99]，默认 P88
	double featureAnglePercentile = 0.88;

	/// 混合策略：特征广义边二面角（度）
	double hybridFeatureAngleDeg = 60.0;
	int hybridClusterMaxIters = 30;
	double hybridSecondarySampleScale = 10.0;
	double hybridMergeCosHigh = 0.70;
	double hybridMergeCosLowBase = 0.20;
	double hybridMergeCosLowScale = 0.30;
	double hybridSmallRegionRatio = 0.01;
	int hybridSmallRegionMin = 10;
	int hybridSmallRegionMax = 100;
	bool hybridEnableRegionAdjust = true;
	int hybridCollapseValenceSumMax = 6;
	double hybridCollapseLengthRatio = 0.60;
	int hybridRegionAdjustMaxPasses = 10;
	int samplesPerPatchEdge = 16;
	/// UV 目标间距(mm)；≤0 时用固定 samplesPerPatchEdge
	double targetUvSpacingMm = 0.0;
	int minSamplesPerEdge = 4;
	/// ≤0 表示不限制每边最大采样数
	int maxSamplesPerEdge = 0;
	/// 拟合降采样上限(每边分段数)；≤0 表示不降采样上限
	int maxFitGridPerEdge = 9;
	/// 拟合 UV 目标间距(mm)；≤0 时仅受 maxFitGridPerEdge 约束
	double fitUvSpacingMm = 0.0;

	/// AMRTO k_sample：参数域栅格密度系数
	double sampleRateFactor = 2.0;
	int sampleGridMin = 10;
	int sampleGridMax = 2000;
	/// AMRTO k_type_gemodl：控制点密度系数
	double controlPointDensityFactor = 0.5;
	int minControlPointsPerDirection = 6;
	int nurbsDegreeU = 3;
	int nurbsDegreeV = 3;
	MeshSurfaceNurbsFitMode fitMode = MeshSurfaceNurbsFitMode::ApproxFixedCtrlpts;
	/// AMRTO k_construct_grid：1 全域 2 内缩 3 仅内部
	int parameterGridMode = 1;
	double fitEvaluationDelta = 0.05;

	double blendStripWidth = 0.0;

	double fairingEpsilon = 1e-3;
	int fairingMaxIterations = 50;

	double tessellateLinearDeflectionMm = 0.1;

	/// 调和 UV 边界初值：Circular=圆周边界；GeodesicSquare=测地四边 square-border（AMRTO §3）
	MeshSurfaceHarmonicBoundaryMode harmonicBoundaryMode = MeshSurfaceHarmonicBoundaryMode::GeodesicSquare;
	/// 单 patch 调和 UV 最大三角面数；≤0 用内置默认 8000
	int harmonicMaxFaces = 8000;

	/// CGAL SDF 分块种子权重 [0,1]；CgalChartHybrid 或 >0 时启用
	double sdfSeedBlendWeight = 0.35;
	int sdfSegmentCount = 0;

	/// AMRTO IM+GMCG：GMCG 后端（GoldenLoader=金标准 OBJ / Exe=外部工具 / Native=内置）
	MeshSurfaceGmcgBackend gmcgBackend = MeshSurfaceGmcgBackend::Native;
	/// IM 目标 quad 顶点数；0=自动
	int instantMeshesTargetQuads = 0;
	double instantMeshesCreaseAngleDeg = 30.0;
	std::string gmcgExePath;
	std::string gmcgWorkDir;
	std::string amrtoGoldenDataPath;
	std::string amrtoGoldenResultObj;
	std::string instantMeshesExePath;
	/// IM 不可用时回退到 CODE_AMRTO 金标准分块（适用于 data_smooth 等样例）
	bool amrtoFallbackGoldenOnImFailure = false;

	/// 多分辨率拟合（AMRTO Algorithm 2 简化版）
	bool enableMultiResolutionFit = true;
	int multiResolutionLayers = 1;
	double multiResolutionDensityScale = 0.5;

	/// 光顺时冻结边界控制点（multi-res / C² 混合后）
	bool fairingProtectBoundaries = true;
	/// 边界混合带深度；≤0 自动
	int blendStripDepth = 0;
};

enum class MeshSurfaceReconstructStage : int
{
	None = 0,
	Partition,
	Sample,
	Fit,
	BoundaryBlend,
	JunctionBlend,
	Fair,
	Assemble,
};

struct MeshSurfaceReconstructReport
{
	int patchCount = 0;
	int junctionCount = 0;
	double maxDeviationMm = 0.0;
	double globalFairingMetric = 0.0;
	double normalSmoothGapVolume = 0.0;
	bool c2BlendSucceeded = false;
	/// 边界混合：实际处理的相邻 patch 对数
	int boundaryBlendPairCount = 0;
	/// 边界混合：被修改的控制点总数
	int boundaryBlendCtrlPtCount = 0;
	/// 边界混合：控制点最大移动距离（mm）
	double boundaryBlendMaxMoveMm = 0.0;
	/// 交汇混合：实际处理的交汇点数
	int junctionBlendAppliedCount = 0;
	/// 交汇混合：角点控制点最大移动距离（mm）
	double junctionBlendMaxMoveMm = 0.0;

	int inputTriangleCount = 0;
	int repairedTriangleCount = 0;
	int remeshedTriangleCount = 0;
	double remeshTargetEdgeLengthUsedMm = 0.0;
	int totalSamplePoints = 0;
	int bsplinePatchCount = 0;
	int nurbsPatchCount = 0;
	/// 拟合失败时回退为原 mesh 三角面的 patch 数（字段名保留兼容）
	int planeFallbackCount = 0;
	int amrtoHarmonicSampleCount = 0;
	int outputFaceCount = 0;
	double avgFacesPerPatch = 0.0;
	int minFacesPerPatch = 0;
	int maxFacesPerPatch = 0;
	int smallPatchCount = 0;
	int initialRegionCount = 0;
	int quadPatchCount = 0;
	int triPatchCount = 0;
	int pentPatchCount = 0;
	int hexPatchCount = 0;
	int gridN = 0;
	int gridNuMax = 0;
	int gridNvMax = 0;

	int fitRejectApprox = 0;
	int fitRejectPole = 0;
	int fitRejectFitGrid = 0;
	int fitRejectFullGrid = 0;
	int fitRejectMakeFace = 0;

	double avgCtrlPtsPerPatch = 0.0;
	int totalCtrlPts = 0;
	int multiResolutionReducedCount = 0;
	int geodesicSquareHarmonicCount = 0;
};

class GEOMETRY_ALGORITHM_API MeshSurfaceReconstructSession
{
public:
	explicit MeshSurfaceReconstructSession(std::vector<float> sourceSoup);
	~MeshSurfaceReconstructSession();

	MeshSurfaceReconstructSession(const MeshSurfaceReconstructSession&) = delete;
	MeshSurfaceReconstructSession& operator=(const MeshSurfaceReconstructSession&) = delete;

	MeshSurfaceReconstructStage lastCompleted() const;
	const MeshSurfaceReconstructReport& report() const;
	MeshSurfaceReconstructReport& report();

private:
	friend GEOMETRY_ALGORITHM_API bool buildPartitionColoredMeshSoup(const MeshSurfaceReconstructSession& session,
																	 std::vector<float>& outSoup,
																	 std::vector<float>& outRgbPerVertex,
																	 std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildSamplePointsCloud(const MeshSurfaceReconstructSession& session,
															  std::vector<float>& outXyz, std::vector<float>& outRgba,
															  std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildFitPreviewShape(const MeshSurfaceReconstructSession& session,
															ShapeHandle& outShape, std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool runMeshSurfaceReconstructStage(MeshSurfaceReconstructSession& session,
																	  MeshSurfaceReconstructStage stage,
																	  const MeshSurfaceReconstructParams& params,
																	  ShapeHandle* outShape, std::string* errMsg);

	struct Impl;
	std::unique_ptr<Impl> m_impl;
};

using MeshSurfaceReconstructSessionPtr = std::shared_ptr<MeshSurfaceReconstructSession>;

GEOMETRY_ALGORITHM_API MeshSurfaceReconstructSessionPtr
createMeshSurfaceReconstructSession(std::vector<float> sourceSoup);

GEOMETRY_ALGORITHM_API bool runMeshSurfaceReconstructStage(MeshSurfaceReconstructSession& session,
														   MeshSurfaceReconstructStage stage,
														   const MeshSurfaceReconstructParams& params,
														   ShapeHandle* outShape, std::string* errMsg = nullptr);

/// 分块完成后按片着色三角 soup（rgb 与 soup 同布局，每顶点 3 float）
GEOMETRY_ALGORITHM_API bool buildPartitionColoredMeshSoup(const MeshSurfaceReconstructSession& session,
														  std::vector<float>& outSoup,
														  std::vector<float>& outRgbPerVertex,
														  std::string* errMsg = nullptr);

/// 采样完成后按片着色栅格采样点（xyz 3 float / 点，rgba 4 float / 点）
GEOMETRY_ALGORITHM_API bool buildSamplePointsCloud(const MeshSurfaceReconstructSession& session,
												   std::vector<float>& outXyz, std::vector<float>& outRgba,
												   std::string* errMsg = nullptr);

/// 拟合完成后组装各 patch 面为 Compound，供场景预览
GEOMETRY_ALGORITHM_API bool buildFitPreviewShape(const MeshSurfaceReconstructSession& session, ShapeHandle& outShape,
												 std::string* errMsg = nullptr);

/**
 * 三角网格 soup → C² 拼接 B 样条 B-rep（单位 mm）
 * 预处理（法矢光顺/修复）由 Data 层在调用前完成
 */
GEOMETRY_ALGORITHM_API bool reconstructBrepFromMeshSoup(const std::vector<float>& soup,
														const MeshSurfaceReconstructParams& params,
														ShapeHandle& outShape, MeshSurfaceReconstructReport& report,
														std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHSURFACERECONSTRUCTION_H
