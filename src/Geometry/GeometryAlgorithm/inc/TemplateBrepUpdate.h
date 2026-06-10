#pragma once

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <Eigen/Geometry>

#include <cstddef>
#include <string>
#include <vector>

namespace geoalgo
{

enum class FaceUpdateAction
{
	Unchanged,
	PlaneRefit,
	CylinderRefit,
	FreeformRefit,
	SkippedNoPoints,
	ConeRefit,
	SphereRefit,
	ToroidRefit,
	PlaneAdjusted,
	CylinderAdjusted,
	ConeAdjusted,
	SphereAdjusted,
	ToroidAdjusted,
	BSplineAdjusted
};

struct FaceUpdateReport
{
	int faceIndex = -1;
	FaceUpdateAction action = FaceUpdateAction::Unchanged;
	std::size_t assignedPoints = 0U;
	double avgDeviationMm = 0.0;
	double maxDeviationMm = 0.0;
	/// OCCT 面类型名称（Plane/Cylinder/Cone/Sphere/Torus/BSplineSurface/...），用于调试
	std::string surfaceTypeName;
};

struct TemplateBrepUpdateParams
{
	double voxelPrefilterMm = 1.0;
	double outlierRemovalPercent = 0.0;
	double icpMaxPairDistanceMm = 0.0;
	std::size_t icpMaxPoints = 8000U;
	double faceBandMm = 2.0;
	double normalThresholdDeg = 35.0;
	std::size_t minPointsPerFace = 30U;
	double maxAllowedDeviationMm = 0.5;
	double sampleSpacingMm = 2.0;
	/// 扫描点已通过 3D 视图变换到 CAD 模型坐标系时设为 true，跳过包围盒中心对齐
	bool scanAlreadyInTemplateFrame = false;
	/// ICP RMSE 超过 max(faceBand*ratio, minIcpRmseGateMm) 时拒绝面更新
	double maxIcpRmseToFaceBandRatio = 8.0;
	double minIcpRmseGateMm = 12.0;
	/// 粗配准阶段启用 FPFH + RANSAC（失败时回退到粗 ICP）
	bool enableRansacCoarseMatch = true;
	int ransacMaxIterations = 5000;
	/// 用户选择的面索引（0-based）；为空时处理所有面
	std::vector<int> selectedFaceIndices;
	/// BSpline 调整：扫描点到原面距离超过此值才驱动控制点；0=固定默认 0.5mm（与 maxAllowedDeviationMm 无关）
	double bsplineAdjustThresholdMm = 0.0;
	/// BSpline 单控制点最大位移；0=自动 max(3*threshold, 1.0) mm
	double bsplineMaxPoleMoveMm = 0.0;
	/// 面点归属预算：选择性重构时每面最多扫描多少点
	std::size_t maxAssignPointsPerFace = 800U;
	/// BSpline UV 聚合网格（越大越平滑、越快）
	int bsplineUvGridCellsU = 24;
	int bsplineUvGridCellsV = 12;
	/// 控制点位移场 Laplacian 平滑迭代
	int bsplinePoleSmoothPasses = 2;
};

struct TemplateBrepUpdateResult
{
	ShapeHandle updatedShape;
	Eigen::Isometry3d scanToTemplate = Eigen::Isometry3d::Identity();
	double icpRmseMm = 0.0;
	std::vector<FaceUpdateReport> perFace;
	double globalMaxDeviationMm = 0.0;
	double globalAvgDeviationMm = 0.0;
	std::size_t updatedFaceCount = 0U;
	/// 因单面或试应用后全局包围盒守卫而跳过的面数
	std::size_t skippedBadBboxFaceCount = 0U;
	bool qualityPassed = false;
};

GEOMETRY_ALGORITHM_API bool sampleShapeSurfacePoints(
	const ShapeHandle& templateShape,
	double spacingMm,
	std::vector<float>& outXyz,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool updateShapeFromPointCloud(
	const ShapeHandle& templateShape,
	const std::vector<float>& scanXyz,
	const std::vector<float>& scanNormalsNxNyNz,
	const TemplateBrepUpdateParams& params,
	TemplateBrepUpdateResult& out,
	std::string* errMsg = nullptr);

} // namespace geoalgo
