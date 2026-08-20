#ifndef GEOMETRYALGORITHM_MESHTRAJECTORY_H
#define GEOMETRYALGORITHM_MESHTRAJECTORY_H

/// @file MeshTrajectory.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 三角 soup 上生成机器人 RawPath：截面法平面求交或 B 样条区域 UV 采样

#include "geometry_algorithm_global.h"

#include "FeatureSpec.h"
#include "MeshSurfaceReconstruction.h"

#include <string>
#include <vector>

namespace geoalgo
{
enum class MeshTrajectoryMethod
{
	CrossSection,  ///< 平面 ∩ 网格 → 弧长重采样
	BsplineRegion  ///< 选中区域拟合 B 样条 → UV 栅格/蛇形
};

enum class MeshTrajectoryUvTraceMode
{
	USerpentine,
	VSerpentine,
	UvGrid
};

struct MeshTrajectoryWorkpiece
{
	std::string backendIdUtf8;
	std::string frameId = "workpiece";
};

struct MeshTrajectoryRegion
{
	std::vector<int> triangleIndices; ///< BsplineRegion 必选 ≥3；CrossSection 可选过滤求交三角
};

struct MeshTrajectoryCrossSection
{
	double planeOriginMm[3]{0.0, 0.0, 0.0}; ///< 截面平面原点（模型 mm）
	double planeNormal[3]{0.0, 0.0, 1.0};    ///< 截面法向（不必单位化，内部归一）
};

struct MeshTrajectoryBsplineParams
{
	int uvCountU = 16;                    ///< 拟合格网基数，≥4
	int uvCountV = 16;
	double gridAngleDeg = 0.0;            ///< 区域 UV 系内扫描方向旋转（°）
	double fitUvSpacingMm = 0.0;          ///< >0 时按 UV 跨度自动收紧格点数
	MeshTrajectoryUvTraceMode traceMode = MeshTrajectoryUvTraceMode::USerpentine;
	MeshSurfaceNurbsFitMode fitMode = MeshSurfaceNurbsFitMode::ApproxCentripetalFixedCtrlpts;
	double controlPointDensityFactor = 0.5;
	int nurbsDegreeU = 3;
	int nurbsDegreeV = 3;
};

struct MeshTrajectorySpec
{
	int schemaVersion = 1;
	std::string trajectoryId;
	MeshTrajectoryWorkpiece workpiece;
	MeshTrajectoryMethod method = MeshTrajectoryMethod::CrossSection;
	MeshTrajectoryRegion region;
	MeshTrajectoryCrossSection crossSection;
	DiscretizeParams discretize;          ///< CrossSection 用 stepMm；BsplineRegion 不用 stepMm
	MeshTrajectoryBsplineParams bspline;
};

struct MeshTrajectoryPolyline
{
	std::vector<RawPathPoint> points;
	bool closed = false;
};

/**
 * 校验 spec 字段完整性
 * @return false：backendId 空、Bspline 未选区、截面法向无效等
 */
GEOMETRY_ALGORITHM_API bool validateMeshTrajectorySpec(const MeshTrajectorySpec& spec, std::string* errMsg = nullptr);

/** 按 triangleIndices 过滤 soup，outOriginalTriangleIndices 记录原索引 */
GEOMETRY_ALGORITHM_API bool filterSoupByTriangleIndices(const std::vector<float>& triangleSoup,
														const std::vector<int>& triangleIndices,
														std::vector<float>& outSoup,
														std::vector<int>& outOriginalTriangleIndices);

/**
 * 平面与三角 soup 求交 → 多条折线
 * @param triangleIndexFilter 非空时仅处理选中三角（边界处交线会断开）
 * @return false：soup 空或无交线
 */
GEOMETRY_ALGORITHM_API bool
intersectPlaneWithTriangleSoup(const std::vector<float>& triangleSoup, const double planeOriginMm[3],
							   const double planeNormalUnit[3], const std::vector<int>* triangleIndexFilter,
							   std::vector<MeshTrajectoryPolyline>& outPolylines, std::string* errMsg = nullptr);

/**
 * 单折线按 stepMm 弧长重采样；法向默认取 planeNormalUnit
 * @return false：点数不足
 */
GEOMETRY_ALGORITHM_API bool discretizeMeshTrajectoryPolyline(MeshTrajectoryPolyline& polyline, double stepMm,
															 bool outputTangent, bool outputNormal,
															 const double planeNormalUnit[3]);

/**
 * 按 spec.method 分派生成 RawPath
 * CrossSection：全部交线段均离散，segmentEndExclusive 标记段界
 * @param triangleSoup 模型坐标 mm，9T float
 * @return false：校验失败、无交线、B 样条拟合失败等
 */
GEOMETRY_ALGORITHM_API bool generateMeshTrajectory(const MeshTrajectorySpec& spec,
												   const std::vector<float>& triangleSoup, RawPath& outPath,
												   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool meshTrajectorySpecFromJson(const std::string& jsonUtf8, MeshTrajectorySpec& out,
													   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool meshTrajectorySpecToJson(const MeshTrajectorySpec& spec, std::string& outJsonUtf8);

/**
 * B 样条区域拟合曲面三角化预览（16~48 细分）
 * @param outTriangleSoupModel 模型 mm，9T float
 * @return false：区域 <3 三角、UV 跨度太小或拟合失败
 */
GEOMETRY_ALGORITHM_API bool buildBsplineRegionSurfacePreview(const std::vector<float>& triangleSoup,
															 const MeshTrajectoryRegion& region,
															 const MeshTrajectoryBsplineParams& bspline,
															 std::vector<float>& outTriangleSoupModel,
															 std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHTRAJECTORY_H
