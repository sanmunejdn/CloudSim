#ifndef GEOMETRYALGORITHM_TEMPLATEBREPUPDATE_H
#define GEOMETRYALGORITHM_TEMPLATEBREPUPDATE_H

/// @file TemplateBrepUpdate.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 扫描点云驱动模板 B-rep 逐面调整（Plane/Cylinder/Cone/Sphere/Toroid/BSpline）

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"

#include <cstddef>
#include <string>
#include <vector>

#include <Eigen/Geometry>

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
	std::string surfaceTypeName; ///< OCCT 曲面类型名，调试用
};

enum class TemplateBrepRegistrationStage
{
	Full = 0,      ///< 粗配 + 精配一步
	CoarseOnly = 1,
	FineOnly = 2,  ///< 需 checkpoint
};

/** 粗配完成后保存，供精配门控与增量链抵消使用；精配 soup 按当前模板 worldMatrix 重建，不在此缓存 */
struct TemplateBrepRegistrationCheckpoint
{
	Eigen::Isometry3d icpDeltaWorld = Eigen::Isometry3d::Identity();
	bool valid = false;
};

struct TemplateBrepUpdateParams
{
	double voxelPrefilterMm = 1.0;
	double outlierRemovalPercent = 0.0;
	double icpMaxPairDistanceMm = 0.0;     ///< 0=自动
	std::size_t icpMaxPoints = 8000U;
	double faceBandMm = 2.0;               ///< 点面归属带宽（mm）
	double normalThresholdDeg = 35.0;
	std::size_t minPointsPerFace = 30U;
	double maxAllowedDeviationMm = 0.5;
	double sampleSpacingMm = 2.0;          ///< 模板面采样间距（mm）
	double maxIcpRmseToFaceBandRatio = 8.0;
	double minIcpRmseGateMm = 12.0;
	bool enableRansacCoarseMatch = true;
	int ransacMaxIterations = 5000;
	TemplateBrepRegistrationStage registrationStage = TemplateBrepRegistrationStage::Full;
	double registrationMatchVoxelMm = 0.0; ///< 0=未启用
	std::vector<int> selectedFaceIndices;  ///< 空=全部面
	double bsplineAdjustThresholdMm = 0.0; ///< 0=默认 0.5mm
	double bsplineMaxPoleMoveMm = 0.0;   ///< 0=自动 max(3×threshold, 1.0)
	std::size_t maxAssignPointsPerFace = 800U;
	int bsplineUvGridCellsU = 24;
	int bsplineUvGridCellsV = 12;
	int bsplinePoleSmoothPasses = 2;
};

struct TemplateBrepUpdateResult
{
	ShapeHandle updatedShape;
	/// 正典字段：newTemplateWorld = icpDeltaWorld × templateWorld。
	/// 例外：FineOnly 且粗配后用户拖过模板时为增量链值 fine×coarse（精配 soup 按当前 worldMatrix 重建，
	/// fine 相对当前帧），正典消费方式是 report.icpDeltaWorld × inv(checkpoint.icpDeltaWorld) 左乘当前世界
	Eigen::Isometry3d icpDeltaWorld = Eigen::Isometry3d::Identity();
	Eigen::Isometry3d templateToScan = Eigen::Isometry3d::Identity(); ///< 遗留别名：与 icpDeltaWorld 恒同值，仅为兼容保留，新代码勿消费
	double icpRmseMm = 0.0;
	bool icpRmseGatePassed = false;
	double registrationOverlapMaxDevMm = 0.0;
	bool registrationPreviewOk = false;
	std::vector<FaceUpdateReport> perFace;
	double globalMaxDeviationMm = 0.0;
	double globalAvgDeviationMm = 0.0;
	std::size_t updatedFaceCount = 0U;
	std::size_t skippedBadBboxFaceCount = 0U; ///< 单面/试应用 bbox 守卫跳过
	bool qualityPassed = false;
};

/**
 * 在 TopoDS_Face 参数域按 spacingMm 网格采样
 * @param outXyz STEP 模型坐标 mm
 * @return false：shape 无效或无面
 */
GEOMETRY_ALGORITHM_API bool sampleShapeSurfacePoints(const ShapeHandle& templateShape, double spacingMm,
													 std::vector<float>& outXyz, std::string* errMsg = nullptr);

/**
 * 扫描驱动模板更新：配准 → 并行点归属 → 逐面 adjustFaceGeometryDispatch → 增量试 Apply
 * @param scanXyz/scanNormals 扫描点云（STEP 模型 mm，与模板同系）
 * @return false：模板/扫描点过少、shape 访问失败或 updatedShape 构建失败
 */
GEOMETRY_ALGORITHM_API bool updateShapeFromPointCloud(const ShapeHandle& templateShape,
													  const std::vector<float>& scanXyz,
													  const std::vector<float>& scanNormalsNxNyNz,
													  const TemplateBrepUpdateParams& params,
													  TemplateBrepUpdateResult& out, std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_TEMPLATEBREPUPDATE_H
