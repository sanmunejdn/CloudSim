#ifndef GEOMETRYALGORITHM_TUBULARGRINDING_H
#define GEOMETRYALGORITHM_TUBULARGRINDING_H

/// @file TubularGrinding.h
/// @brief 管状铸件：中心线 → 模板轨迹点 → 表面投影（网格/点云双源，会话式分阶段）

#include "geometry_algorithm_global.h"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace geoalgo
{
enum class TubularGrindingStage : int
{
	None = 0,
	Segment = 1,
	Centerline = 2,
	TemplatePoints = 3,
	Project = 4,
	FpfhRegionPartition = 5,
};

enum class TubularGrindingTemplateKind : int
{
	Helical = 0,
	Circumferential = 1,
	AxialParallel = 2,
	Zigzag = 3,
	Auto = 4,
};

/// 邻域搜索模式
enum class NeighborhoodMode : int
{
	Fixed2Hop = 0, // 原有固定 2-hop（兼容）
	Adaptive = 1,  // 自适应测地线距离
};

/// 截面拟合模式
enum class SectionFitMode : int
{
	Circle = 0,		// 原有圆拟合（兼容）
	Ellipse = 1,	// 椭圆拟合
	ConvexHull = 2, // 凸包拟合（通用回退）
};

enum class TubularGrindingCenterlineMethod : int
{
	Laplacian = 0,
	OtLc = 1,
};

struct TubularGrindingParams
{
	/// 环心 DBSCAN 半径（mm）；0 表示按网格尺度自动估计
	double ringCenterClusterEpsMm = 0.0;
	/// 有效环最少三角面数
	double minRingFaces = 4.0;
	/// 环链合并：相邻环轴线夹角上限（°）
	double axisMergeAngleDeg = 28.0;
	/// 交汇环判定：邻接环数 ≥ 3 且轴线散布超过该值（°）
	double junctionAxisSpreadDeg = 38.0;
	/// 有效管段最少三角面数
	double minSegmentFaces = 40.0;
	/// 法向射线汇聚容差（mm）；0 表示按局部管径自动放宽
	double ringRayConvergenceEpsMm = 0.0;
	/// 法向轴可视化长度（mm）；0 表示按网格尺度自动
	double faceNormalAxisLengthMm = 0.0;
	/// 保留兼容
	double regionGrowAxisAngleDeg = 28.0;

	double sectionSpacingMm = 2.0;
	/// 局部轴线估计最少邻居面数（原截面圆拟合最少点数）
	int minSectionPoints = 4;

	TubularGrindingTemplateKind templateKind = TubularGrindingTemplateKind::Auto;
	int helicalCoils = 8;              ///< 螺旋模板圈数
	int circumferentialRings = 30;     ///< 环形模板截面数
	int axialMeridians = 24;           ///< 轴向平行模板经线数
	int zigzagPasses = 40;             ///< 锯齿模板 pass 数

	double projectionMaxDistMm = 10.0; ///< 模板点沿 ±法向投影最大搜索半径（mm）

	// === 广义管状分析新增参数 ===

	/// 邻域搜索模式
	NeighborhoodMode neighborhoodMode = NeighborhoodMode::Adaptive;
	/// 测地线搜索半径（mm）；0 表示自动估计
	double geodesicRadiusMm = 0.0;
	/// 截面拟合模式
	SectionFitMode sectionFitMode = SectionFitMode::Ellipse;
	/// 长短轴比突变阈值，过渡区检测
	double transitionAspectRatioThreshold = 0.3;
	/// 中心线曲率突变阈值（°），过渡区检测
	double transitionCurvatureThresholdDeg = 15.0;
	/// 中心线迭代平滑次数（Laplacian 收缩 + 拓扑塌缩迭代次数）
	int centerlineIterations = 80;
	/// 中心线迭代收敛阈值（mm）（已废弃，保留向后兼容）
	double centerlineConvergenceEpsMm = 0.01;

	// === 拉普拉斯收缩新增参数 ===
	/// 收缩强度：映射为中期锚定权重峰值（10–200，越大收得越快）
	double laplacianLambda = 0.1;
	/// 初始锚定强度（越大前期越稳）
	double laplacianAttraction = 0.2;
	/// KNN 邻域大小
	int laplacianKNeighbors = 8;

	TubularGrindingCenterlineMethod centerlineMethod = TubularGrindingCenterlineMethod::Laplacian;

	double otSampleRate = 0.10;        ///< OTLC 体素降采样比例；越小 sample 越少
	double otCostBeta = 3.0;           ///< OT 代价距离指数
	int otcPreSteps = 3;               ///< OTLC 预处理 OT+合并轮次
	int otcOuterLoops = 3;             ///< 每轮外循环内 OT 次数
	int otLcOuterMaxIters = 40;        ///< OTLC 外循环上限
	int pointCloudKnnK = 30;           ///< 点云 KNN 邻域大小

	/// 根点合并下限（0 = 自动：max(40, sampleCount×0.15)）
	int minRootsBySamples = 0;

	// FPFH 区域划分
	double fpfhFeatureVoxelMm = 0.0;           ///< FPFH 特征体素（mm）；0=自动
	int fpfhMaxSamplePoints = 0;               ///< 特征计算采样上限；0=不限
	unsigned int fpfhNeighbors = 20U;          ///< FPFH 邻域点数
	unsigned int fpfhSaliencyNeighbors = 10U;
	int fpfhKeypointCount = 0;                 ///< 关键点数；0=自动
	double fpfhKeypointMinSeparationMm = 0.0;
	double fpfhRegionGrowDist = 0.0;           ///< 区域生长距离（mm）；0=自动
	double fpfhRegionGrowNormalAngleDeg = 45.0;
	int fpfhMinRegionFaces = 10;
};

struct TubularPipeSegment
{
	int id = 0;
	std::vector<int> faceIndices;
	std::array<double, 3> axisHint{{0.0, 0.0, 1.0}};
};

struct TubularCrossSectionRing
{
	int id = 0;
	std::vector<int> faceIndices;
	std::array<double, 3> centerMm{{0.0, 0.0, 0.0}};
	std::array<double, 3> axisHint{{0.0, 0.0, 1.0}};
	double radiusMm = 0.0;
	/// 截面参数（椭圆拟合结果）
	double semiMajorMm = 0.0;
	double semiMinorMm = 0.0;
	double sectionRotationDeg = 0.0;
	double aspectRatio = 1.0;
	/// 是否为过渡区（变径、弯头、连接件）
	bool isTransition = false;
};

struct TubularCenterlineSample
{
	int pipeId = 0;
	double arcLengthMm = 0.0;
	double radiusMm = 0.0;
	std::array<double, 3> positionMm{{0.0, 0.0, 0.0}};
	std::array<double, 3> tangent{{0.0, 0.0, 1.0}};
	std::array<double, 3> normal{{1.0, 0.0, 0.0}};
	std::array<double, 3> binormal{{0.0, 1.0, 0.0}};
	/// 截面参数（椭圆拟合结果）
	double semiMajorMm = 0.0;
	double semiMinorMm = 0.0;
	double sectionRotationDeg = 0.0;
	double aspectRatio = 1.0;
};

/// 中心线 Stage C 所用全局 PCA（收缩后点云的主轴，供调试可视化）
struct TubularCenterlinePcaAxis
{
	std::array<double, 3> centroidMm{{0.0, 0.0, 0.0}};
	std::array<double, 3> axis{{1.0, 0.0, 0.0}};
	double extentMinMm = 0.0;
	double extentMaxMm = 0.0;
	bool valid = false;
};

struct TubularTemplatePoint
{
	int pipeId = 0;
	double paramT = 0.0;
	std::array<double, 3> positionMm{{0.0, 0.0, 0.0}};
	std::array<double, 3> normalMm{{0.0, 0.0, 1.0}};
};

struct TubularProjectedPoint
{
	int pipeId = 0;
	std::array<double, 3> positionMm{{0.0, 0.0, 0.0}};
	std::array<double, 3> normalMm{{0.0, 0.0, 1.0}};
};

struct TubularGrindingProjectedPoints
{
	std::vector<TubularProjectedPoint> points;
};

struct TubularGrindingReport
{
	int pipeCount = 0;
	int ringCount = 0;
	int junctionFaceCount = 0;
	int regionCountBeforeFilter = 0;
	int centerlinePointCount = 0;
	int templatePointCount = 0;
	int projectedPointCount = 0;
	int sectionFitFailCount = 0;
	double projectionHitRate = 0.0;
	/// OTLC 中心线是否走了 PCA 分箱兜底（false 表示 OT sample 图最长路径）
	bool centerlinePcaFallback = false;
	/// 中心线是否由 OTLC 算法提取（用于摘要区分 Laplacian）
	bool centerlineOtLcExtraction = false;
	/// OTLC 提线调试：活跃 sample 根数
	int centerlineOtRootCount = 0;
	/// OTLC 提线调试：sample 图无向边数
	int centerlineOtEdgeCount = 0;
	/// OTLC 提线调试：sample 图连通分量数
	int centerlineOtComponentCount = 0;
	/// OTLC 提线是否用 KNN 补边（sampleEdges 为空时）
	bool centerlineOtKnnFallbackEdges = false;
	/// OTLC 提线路径：0=收缩点云截面质心(全局PCA) 1=OT 分簇链 2=有序折线兜底
	int centerlineOtPathKind = 0;
	/// 过渡区面数
	int transitionFaceCount = 0;
	/// 中心线迭代收敛次数
	int centerlineIterationCount = 0;
	int fpfhRegionCount = 0;
	int fpfhKeypointCount = 0;
};

class GEOMETRY_ALGORITHM_API TubularGrindingSession
{
public:
	explicit TubularGrindingSession(std::vector<float> sourceSoup);
	/// 内部使用：点云路径构造（isPointCloud 仅作标记）
	TubularGrindingSession(std::vector<float> pointXyz, bool isPointCloud);
	~TubularGrindingSession();

	TubularGrindingSession(const TubularGrindingSession&) = delete;
	TubularGrindingSession& operator=(const TubularGrindingSession&) = delete;

	TubularGrindingStage lastCompleted() const;
	const TubularGrindingReport& report() const;
	TubularGrindingReport& report();
	const std::vector<TubularPipeSegment>& pipeSegments() const;
	const std::vector<TubularCrossSectionRing>& crossSectionRings() const;
	const std::vector<TubularCenterlineSample>& centerlineSamples() const;
	const std::vector<TubularTemplatePoint>& templatePoints() const;
	const std::vector<TubularProjectedPoint>& projectedPoints() const;

private:
	friend GEOMETRY_ALGORITHM_API bool runTubularGrindingStage(TubularGrindingSession& session,
															   TubularGrindingStage stage,
															   const TubularGrindingParams& params,
															   std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildSegmentColoredMeshSoup(const TubularGrindingSession& session,
																   std::vector<float>& outSoup,
																   std::vector<float>& outRgbPerVertex,
																   std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildRingColoredMeshSoup(const TubularGrindingSession& session,
																std::vector<float>& outSoup,
																std::vector<float>& outRgbPerVertex,
																std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildFpfhRegionColoredMeshSoup(const TubularGrindingSession& session,
																	  std::vector<float>& outSoup,
																	  std::vector<float>& outRgbPerVertex,
																	  std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildRingCenterPointsCloud(const TubularGrindingSession& session,
																  std::vector<float>& outXyz,
																  std::vector<float>& outRgba, std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildFaceNormalAxisLineSegments(const TubularGrindingSession& session,
																	   const TubularGrindingParams& params,
																	   std::vector<float>& outLineXyz,
																	   std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildLocalAxisLineSegments(const TubularGrindingSession& session,
																  const TubularGrindingParams& params,
																  std::vector<float>& outLineXyz, std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildCenterlinePointsCloud(const TubularGrindingSession& session,
																  std::vector<float>& outXyz,
																  std::vector<float>& outRgba, std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildCenterlinePolylineXyz(const TubularGrindingSession& session,
																  std::vector<float>& outXyz, std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildCenterlinePcaAxisArrowLineSegments(const TubularGrindingSession& session,
																			   std::vector<float>& outLineXyz,
																			   std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildTemplatePointsCloud(const TubularGrindingSession& session,
																std::vector<float>& outXyz, std::vector<float>& outRgba,
																std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildProjectedPointsCloud(const TubularGrindingSession& session,
																 std::vector<float>& outXyz,
																 std::vector<float>& outRgba, std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool buildIterationSnapshotPointsCloud(const TubularGrindingSession& session,
																		 int snapshotIndex, std::vector<float>& outXyz,
																		 std::vector<float>& outRgba,
																		 std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API bool
	buildIterationSnapshotContractedPointsCloud(const TubularGrindingSession& session, int snapshotIndex,
												std::vector<float>& outXyz, std::vector<float>& outRgba,
												std::string* errMsg);

	friend GEOMETRY_ALGORITHM_API int iterationSnapshotCount(const TubularGrindingSession& session);

	friend GEOMETRY_ALGORITHM_API int iterationSnapshotIteration(const TubularGrindingSession& session,
																 int snapshotIndex);

	friend GEOMETRY_ALGORITHM_API bool computeEllipseFittingResidualReport(const TubularGrindingSession& session,
																		   const TubularGrindingParams& params,
																		   std::vector<double>& outPerRingRmsResiduals,
																		   std::string& outSummaryText,
																		   std::string* errMsg);

	struct Impl;
	std::unique_ptr<Impl> m_impl;
};

using TubularGrindingSessionPtr = std::shared_ptr<TubularGrindingSession>;

/** 从三角 soup（mm，9T）创建会话 */
GEOMETRY_ALGORITHM_API TubularGrindingSessionPtr createTubularGrindingSession(std::vector<float> sourceSoup);

/** 从点云 xyz（3N mm）创建会话；Segment/Laplacian 需 mesh，点云中心线走 OtLc */
GEOMETRY_ALGORITHM_API TubularGrindingSessionPtr
createTubularGrindingSessionFromPointCloud(std::vector<float> pointXyz);

/**
 * 按 stage 顺序执行：Segment(可选) → Centerline → TemplatePoints → Project
 * @param params 见 TubularGrindingParams；sectionSpacingMm 同时用于 PCA 分箱与输出采样
 * @return false：阶段乱序、mesh 拓扑缺失、中心线提取失败（Laplacian/OTLC）或前置阶段未完成
 */
GEOMETRY_ALGORITHM_API bool runTubularGrindingStage(TubularGrindingSession& session, TubularGrindingStage stage,
													const TubularGrindingParams& params, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildSegmentColoredMeshSoup(const TubularGrindingSession& session,
														std::vector<float>& outSoup,
														std::vector<float>& outRgbPerVertex,
														std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildRingColoredMeshSoup(const TubularGrindingSession& session, std::vector<float>& outSoup,
													 std::vector<float>& outRgbPerVertex,
													 std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildFpfhRegionColoredMeshSoup(const TubularGrindingSession& session,
														   std::vector<float>& outSoup,
														   std::vector<float>& outRgbPerVertex,
														   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildRingCenterPointsCloud(const TubularGrindingSession& session,
													   std::vector<float>& outXyz, std::vector<float>& outRgba,
													   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildFaceNormalAxisLineSegments(const TubularGrindingSession& session,
															const TubularGrindingParams& params,
															std::vector<float>& outLineXyz,
															std::string* errMsg = nullptr);

/// 构建 Phase 1 计算的局部轴线线段（双向，正方向青色/反方向品红）
GEOMETRY_ALGORITHM_API bool buildLocalAxisLineSegments(const TubularGrindingSession& session,
													   const TubularGrindingParams& params,
													   std::vector<float>& outLineXyz, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildCenterlinePointsCloud(const TubularGrindingSession& session,
													   std::vector<float>& outXyz, std::vector<float>& outRgba,
													   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildCenterlinePolylineXyz(const TubularGrindingSession& session,
													   std::vector<float>& outXyz, std::string* errMsg = nullptr);

/// 中心线 PCA 主轴箭头（overlay 线段，6 float/段）
GEOMETRY_ALGORITHM_API bool buildCenterlinePcaAxisArrowLineSegments(const TubularGrindingSession& session,
																	std::vector<float>& outLineXyz,
																	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildTemplatePointsCloud(const TubularGrindingSession& session, std::vector<float>& outXyz,
													 std::vector<float>& outRgba, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool buildProjectedPointsCloud(const TubularGrindingSession& session, std::vector<float>& outXyz,
													  std::vector<float>& outRgba, std::string* errMsg = nullptr);

/// 迭代快照：OT 活跃 sample 根点云（`_迭代N`）
GEOMETRY_ALGORITHM_API bool buildIterationSnapshotPointsCloud(const TubularGrindingSession& session, int snapshotIndex,
															  std::vector<float>& outXyz, std::vector<float>& outRgba,
															  std::string* errMsg = nullptr);

/// 迭代快照：LC 收缩 original 子采样（`_迭代N_收缩`）
GEOMETRY_ALGORITHM_API bool buildIterationSnapshotContractedPointsCloud(const TubularGrindingSession& session,
																		int snapshotIndex, std::vector<float>& outXyz,
																		std::vector<float>& outRgba,
																		std::string* errMsg = nullptr);

/// 迭代快照数量
GEOMETRY_ALGORITHM_API int iterationSnapshotCount(const TubularGrindingSession& session);

/// 迭代快照的迭代编号
GEOMETRY_ALGORITHM_API int iterationSnapshotIteration(const TubularGrindingSession& session, int snapshotIndex);

/// 计算每个环的椭圆拟合残差（RMS），输出摘要文本
GEOMETRY_ALGORITHM_API bool computeEllipseFittingResidualReport(const TubularGrindingSession& session,
																const TubularGrindingParams& params,
																std::vector<double>& outPerRingRmsResiduals,
																std::string& outSummaryText,
																std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_TUBULARGRINDING_H
