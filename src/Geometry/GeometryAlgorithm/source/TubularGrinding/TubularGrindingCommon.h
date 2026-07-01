#pragma once

#include "TubularGrinding.h"

#include <array>
#include <cmath>
#include <cstddef>
#include <functional>
#include <limits>
#include <string>
#include <vector>

namespace geoalgo
{
namespace tg
{

struct Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

struct IndexedMeshLite
{
	std::vector<float> soup;
	int faceCount = 0;
	std::vector<Vec3> faceCentroids;
	std::vector<Vec3> faceNormals;
	std::vector<std::array<int, 3>> faceVerts;
	std::vector<std::vector<int>> faceNeighbors;
	std::array<double, 3> bboxMin{{0.0, 0.0, 0.0}};
	std::array<double, 3> bboxMax{{0.0, 0.0, 0.0}};
};

bool buildIndexedMeshLite(const std::vector<float>& soup, IndexedMeshLite& out, std::string* errMsg);

void orientMeshFaceNormals(IndexedMeshLite& mesh);

bool isPlanarStencilFace(const IndexedMeshLite& mesh, const int faceIndex, const double minNormalSpreadDeg);

bool rayBundleCenterPoint(
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs,
	Vec3& outCenter);

void segmentDisplayRgb(int segmentIndex, int segmentCount, float& outR, float& outG, float& outB);

Vec3 normalizeVec3(const Vec3& v);
double dot(const Vec3& a, const Vec3& b);
Vec3 cross(const Vec3& a, const Vec3& b);
Vec3 add(const Vec3& a, const Vec3& b);
Vec3 sub(const Vec3& a, const Vec3& b);
Vec3 scale(const Vec3& v, const double s);
double length(const Vec3& v);
double clamp01(const double v);

/// 对称 3×3 协方差矩阵最小特征值对应特征向量
bool smallestEigenvector3(
	const double cov[3][3],
	Vec3& outEigenvector);

Vec3 computeLocalAxisFromFaceNormals(const IndexedMeshLite& mesh, const int faceIndex);

/// 基于邻居面法向量叉积估计局部管轴
Vec3 computeLocalAxisFromNormalCrossProducts(
	const IndexedMeshLite& mesh, int faceIndex, int neighborHop);

/// 从法向量叉积推导环心和半径
bool computeFaceCenterFromNormals(
	const IndexedMeshLite& mesh, int faceIndex,
	double convergenceEpsMm, Vec3& outCenter, double& outRadius);

/// 从多组局部轴线聚合主轴方向
Vec3 computeMainAxisFromFaceAxes(const std::vector<Vec3>& faceAxes);

int runDbscan(
	const std::vector<Vec3>& featurePoints,
	double eps,
	int minPts,
	std::vector<int>& outLabels);

double axisAngleDeg(const Vec3& a, const Vec3& b);

bool rayBundleCenterPoint(
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs,
	Vec3& outCenter);

/// 射线束近似汇聚点；maxMeanDistanceMm≤0 时按局部尺度自动放宽
bool approximateRayBundleCenter(
	const std::vector<Vec3>& origins,
	const std::vector<Vec3>& inwardDirs,
	double maxMeanDistanceMm,
	Vec3& outCenter);

bool fitCircle2d(
	const std::vector<std::array<double, 2>>& pts,
	double& outCx,
	double& outCy,
	double& outRadius);

void buildFrenetFrames(
	const std::vector<TubularCenterlineSample>& samples,
	std::vector<TubularCenterlineSample>& outSamples);

bool computeCenterlinePcaAxisFromPoints(
	const std::vector<Vec3>& points,
	TubularCenterlinePcaAxis& outPca);

TubularGrindingTemplateKind selectTemplateKind(
	const TubularPipeSegment& segment,
	const std::vector<TubularCenterlineSample>& samples);

// === 广义管状分析新增函数 ===

/// 自适应邻域：基于测地线距离 + 局部曲率动态调整搜索范围
/// @param targetGeodesicRadiusMm 搜索半径；0 = 自动估计
/// @param outGeodesicDistances 输出每个邻居的测地线距离
/// @return 邻居面索引列表（含自身）
std::vector<int> collectAdaptiveNeighborhood(
	const IndexedMeshLite& mesh,
	int faceIndex,
	double targetGeodesicRadiusMm,
	std::vector<double>& outGeodesicDistances);

/// 加权PCA：邻域法向量协方差分析，最小特征向量即局部轴线
/// 高斯权重 w_i = exp(-d_i^2 / sigma^2)，sigma 为邻域测地线距离中位数
Vec3 computeLocalAxisFromWeightedPCA(
	const IndexedMeshLite& mesh,
	int faceIndex,
	const std::vector<int>& neighborhood,
	const std::vector<double>& geodesicDistances);

/// 椭圆拟合：最小二乘拟合椭圆参数
/// @return 拟合成功（至少需要 5 个点）
bool fitEllipse2D(
	const std::vector<std::array<double, 2>>& pts,
	double& outSemiMajor,
	double& outSemiMinor,
	double& outCx,
	double& outCy,
	double& outRotationRad);

/// 凸包中心：通用截面的几何中心计算
Vec3 computeConvexHullCentroid2D(
	const std::vector<std::array<double, 2>>& pts);

/// 广义截面分析：切平面投影 + 椭圆/凸包拟合
/// @param localAxis 局部轴线方向
/// @param outSectionParams 输出截面参数
bool analyzeCrossSection(
	const IndexedMeshLite& mesh,
	const std::vector<int>& neighborhood,
	const Vec3& localAxis,
	SectionFitMode fitMode,
	double& outSemiMajor,
	double& outSemiMinor,
	double& outRotationDeg,
	Vec3& outCenter);

/// 扩展DBSCAN：空间坐标 + 截面特征联合聚类
int runDbscanEnhanced(
	const std::vector<Vec3>& spatialPoints,
	const std::vector<double>& semiMajorValues,
	const std::vector<double>& semiMinorValues,
	double eps,
	int minPts,
	double featureScale,
	std::vector<int>& outLabels);

/// 过渡区检测：监控截面参数突变
std::vector<int> detectTransitionZones(
	const IndexedMeshLite& mesh,
	const std::vector<TubularCrossSectionRing>& rings,
	double aspectRatioChangeThreshold,
	double curvatureChangeThresholdDeg);

/// 中心线迭代平滑：B样条平滑 + 迭代优化
bool smoothCenterlineIterative(
	std::vector<TubularCenterlineSample>& samples,
	int maxIterations,
	double convergenceEpsilonMm);

/// 椭圆曲率：参数 t 处的曲率 k(t) = ab / (a^2*sin^2(t) + b^2*cos^2(t))^(3/2)
double ellipseCurvature(double semiMajor, double semiMinor, double tRad);

/// 椭圆弧长参数化：根据曲率自适应调整角度步长
std::vector<double> computeAnisotropicAngleSamples(
	double semiMajor,
	double semiMinor,
	int targetPointCount);

/// 截面法线方向：椭圆参数 t 处的外法线
Vec3 computeSectionNormal(
	double semiMajor,
	double semiMinor,
	double sectionRotationDeg,
	double tRad,
	const Vec3& normalAxis,
	const Vec3& binormalAxis);

/// OTLC 双源输入类型
enum class SkeletonInputKind
{
	Mesh,
	PointCloud
};

/// 统一骨架输入（供 runOtLcSkeletonCenterline 使用）
struct SkeletonInput
{
	SkeletonInputKind kind = SkeletonInputKind::Mesh;
	const IndexedMeshLite* mesh = nullptr;
	const std::vector<float>* pointXyz = nullptr;
};

struct OtLcGraphDiagnostics
{
	int rootSampleCount = 0;
	int undirectedEdgeCount = 0;
	int connectedComponentCount = 0;
	bool usedKnnFallbackEdges = false;
	/// 0=收缩点云截面质心(全局PCA) 1=OT 分簇链 2=有序折线兜底
	int extractPathKind = 0;
};

/// 迭代快照（供 3D 可视化回传）
struct OtLcIterationSnapshot
{
	int iteration = 0;
	/// OT 活跃 sample 根（射线束精炼后）
	std::vector<Vec3> samplePositions;
	/// LC 收缩后的 original 均匀子采样（≤8k）
	std::vector<Vec3> contractedPositions;
};

using OtLcIterationCallback = std::function<void(const OtLcIterationSnapshot&)>;

/// OTLC 主入口（双源）
bool runOtLcSkeletonCenterline(
	const SkeletonInput& input,
	const TubularGrindingParams& params,
	std::vector<TubularCenterlineSample>& outSamples,
	TubularCenterlinePcaAxis* outPcaAxis,
	std::string* errMsg,
	bool* outCenterlinePcaFallback = nullptr,
	OtLcGraphDiagnostics* outGraphDiagnostics = nullptr,
	OtLcIterationCallback onIteration = nullptr);

/// 椭圆拟合残差：计算每个点到拟合椭圆的几何距离
/// @param pts 2D 点集（切平面坐标系）
/// @param semiMajor 长半轴
/// @param semiMinor 短半轴
/// @param cx 椭圆中心 x
/// @param cy 椭圆中心 y
/// @param rotationRad 椭圆旋转角
/// @param outResiduals 输出每个点的残差（点到椭圆最近点的距离）
/// @return 残差的均方根 (RMS)
double computeEllipseFittingResiduals(
	const std::vector<std::array<double, 2>>& pts,
	double semiMajor,
	double semiMinor,
	double cx,
	double cy,
	double rotationRad,
	std::vector<double>& outResiduals);

// === 拉普拉斯收缩骨架提取 ===

/// 构建顶点邻接表（KNN 或网格边）
std::vector<std::vector<int>> buildVertexAdjacency(
	const IndexedMeshLite& mesh,
	int kNeighbors = 8);

/// 计算拉普拉斯坐标 L(V) = Σ w_i (V_i - V)
std::vector<Vec3> computeLaplacianCoordinates(
	const std::vector<Vec3>& positions,
	const std::vector<std::vector<int>>& adjacency);

/// Cao 式迭代收缩：w 越大越贴近骨架，越小越锚定原网格
void contractVerticesIterative(
	std::vector<Vec3>& positions,
	const std::vector<Vec3>& originalPositions,
	const std::vector<std::vector<int>>& adjacency,
	int iterations,
	double weightStart,
	double weightEnd);

/// 焊接顶点数（faceVerts 最大索引 + 1）
int countWeldedVertices(const IndexedMeshLite& mesh);

/// 完整 Laplacian 收缩骨架 → 中心线采样
bool runLaplacianSkeletonCenterline(
	const IndexedMeshLite& mesh,
	const TubularGrindingParams& params,
	std::vector<TubularCenterlineSample>& outSamples,
	TubularCenterlinePcaAxis* outPcaAxis = nullptr);

/// 无序点集 → 有序中心线折线（PCA 质心分箱；失败时 KNN 最长路径）
bool extractOrderedCenterlinePolyline(
	const std::vector<Vec3>& points,
	double binWidthMm,
	std::vector<Vec3>& outPolyline);

/// 有序折线弧长重采样
void resamplePolylineToSamples(
	const std::vector<Vec3>& polyline,
	double spacingMm,
	std::vector<TubularCenterlineSample>& outSamples);

/// Cao 式收缩锚定权重调度（OTLC / Laplacian 共用）
double computeContractionAnchorWeight(
	int iteration,
	int totalIterations,
	double weightStart,
	double weightPeak);

/// 点集 + 邻接图最长路径折线
bool extractLongestPathPolylineFromGraph(
	const std::vector<Vec3>& positions,
	const std::vector<std::vector<int>>& adjacency,
	std::vector<Vec3>& outPolyline);

} // namespace tg
} // namespace geoalgo
