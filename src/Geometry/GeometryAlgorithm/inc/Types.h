#ifndef GEOMETRYALGORITHM_TYPES_H
#define GEOMETRYALGORITHM_TYPES_H

/// @file Types.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 几何算法公共类型：点/折线/离散与求交参数

#include "geometry_algorithm_global.h"

#include <cstddef>
#include <string>
#include <vector>

namespace geoalgo
{
struct Point3d
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
};

/** 有序顶点折线，xyz 为 3N float（mm） */
struct Polyline3d
{
	std::vector<float> xyz;
};

/** OCCT BRepMesh 离散精度 */
struct TessellateParams
{
	double linearDeflectionMm = 0.01;       ///< mm；relative=true 时为相对包围盒比例
	bool linearDeflectionRelative = true;
	double angularDeflectionDeg = 0.5;      ///< °
	bool flipReversedFaces = true;          ///< REVERSED 面翻转三角绕序
};

enum class MeshDiscretizeMode
{
	AdaptiveTriangulation, ///< 默认 STEP 路径（BRepMesh_IncrementalMesh）
	UniformRelative,
	UVStructuredGrid,
	WireTubeMesh,
	WireRibbonMesh,
	ProfileSweepMesh,      ///< 当前构建未实现
	RemeshSoup,            ///< 当前构建未实现
	PointCloudSurface      ///< 当前构建未实现
};

enum class MeshQualityPreset
{
	Coarse,
	Medium,
	Fine,
	Custom
};

/** 与 quality 预设互斥的密度控制模式 */
enum class MeshDensityControl
{
	QualityPreset,
	TargetEdgeLength,      ///< deflection=target×0.25；refine 至 1.5×target
	TargetTriangleCount    ///< 相对 deflection 二分，容差 ±15%
};

struct MeshDiscretizeParams
{
	MeshDiscretizeMode mode = MeshDiscretizeMode::AdaptiveTriangulation;
	MeshQualityPreset quality = MeshQualityPreset::Medium;
	MeshDensityControl densityControl = MeshDensityControl::QualityPreset;
	double targetEdgeLengthMm = 0.0;       ///< TargetEdgeLength 模式（mm）
	std::size_t targetTriangleCount = 0;
	TessellateParams tessellate;
	int uvGridCountU = 32;
	int uvGridCountV = 32;
	double tubeRadiusMm = 1.0;
	int tubeSides = 12;
	double ribbonWidthMm = 2.0;
	bool mergeCoplanarTriangles = false;
};

struct MeshDiscretizeReport
{
	std::size_t triangleCount = 0;
	double bboxDiagonalMm = 0.0;
	double avgEdgeLengthMm = 0.0;
	MeshDiscretizeMode modeUsed = MeshDiscretizeMode::AdaptiveTriangulation;
};

/** 装配层级零件；triangleSoup 可能为空（仅拓扑路径） */
struct MeshHierarchyPart
{
	std::string partPath;
	std::string parentPartPath;
	std::string displayName;
	std::vector<float> triangleSoup;
};

struct PolylineParams
{
	bool closedPreserveEndpoint = false;
};

struct IntersectionHit
{
	Point3d positionMm;
	double paramOnEdge1 = 0.0;
	double paramOnEdge2 = 0.0;
	double paramU = 0.0;
	double paramV = 0.0;
};

struct IntersectionResult
{
	std::vector<IntersectionHit> points;
	std::vector<Polyline3d> curves;
	double maxResidualMm = 0.0;
};

struct IntersectionParams
{
	double toleranceMm = 1e-3;
	bool discretizeCurves = true;
	TessellateParams curveDisc;
};

enum class MeshBooleanOp
{
	Difference,
	Union,
	Intersection
};

enum class BrepBooleanOp
{
	Fuse,
	Common,
	Cut
};

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_TYPES_H
