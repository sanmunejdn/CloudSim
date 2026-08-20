#ifndef GEOMETRYALGORITHM_ASSEMBLYMATE_H
#define GEOMETRYALGORITHM_ASSEMBLYMATE_H

/// @file AssemblyMate.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 装配一次定位：面几何查询与刚体增量（同坐标系 mm）

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"
#include "Types.h"

#include <string>

#include <Eigen/Geometry>

namespace geoalgo
{
enum class FaceMateSurfaceKind
{
	Plane,
	Cylinder,
	Cone,
	Sphere,
	Torus,
	Other
};

struct FaceMateGeom
{
	FaceMateSurfaceKind kind = FaceMateSurfaceKind::Other;
	Point3d originMm;   ///< 平面上一点 / 轴点 / 球心
	Point3d axisUnit;   ///< 平面外向法向或回转轴
	double radiusMm = 0.0;
	double radius2Mm = 0.0; ///< 圆锥半角(rad) 或圆环小半径
	Point3d pickHintMm;
};

enum class AssemblyMateKind
{
	Coincident,
	Parallel,
	Perpendicular,
	Tangent,
	Concentric,
	Lock,
	Distance,
	Angle
};

enum class AssemblyMateAlignment
{
	Aligned,
	AntiAligned
};

struct AssemblyMateParams
{
	AssemblyMateKind kind = AssemblyMateKind::Coincident;
	AssemblyMateAlignment alignment = AssemblyMateAlignment::AntiAligned;
	double distanceMm = 0.0;
	double angleDeg = 90.0;
};

/// faceIndex 相对传入 shape（整件 STEP 是装配 compound 序）
GEOMETRY_ALGORITHM_API bool queryFaceMateGeom(const ShapeHandle& shape, int faceIndex, const Point3d* pickHint,
											  FaceMateGeom& out, std::string* errMsg = nullptr);

/// 两面已在同一坐标系；增量作用于动件点 x' = delta * x
GEOMETRY_ALGORITHM_API bool computeAssemblyMateDelta(const FaceMateGeom& grounded, const FaceMateGeom& moving,
													 const AssemblyMateParams& params, Eigen::Isometry3d& outMovingDelta,
													 std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_ASSEMBLYMATE_H
