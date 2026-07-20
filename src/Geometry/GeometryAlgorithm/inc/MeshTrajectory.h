#ifndef GEOMETRYALGORITHM_MESHTRAJECTORY_H
#define GEOMETRYALGORITHM_MESHTRAJECTORY_H

/// @file MeshTrajectory.h
/// @brief B 样条区域拟合曲面三角化预览（模型坐标 mm，每三角 9 float）

#include "geometry_algorithm_global.h"

#include "FeatureSpec.h"
#include "MeshSurfaceReconstruction.h"

#include <string>
#include <vector>

namespace geoalgo
{
enum class MeshTrajectoryMethod
{
	CrossSection,
	BsplineRegion
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
	std::vector<int> triangleIndices;
};

struct MeshTrajectoryCrossSection
{
	double planeOriginMm[3]{0.0, 0.0, 0.0};
	double planeNormal[3]{0.0, 0.0, 1.0};
};

struct MeshTrajectoryBsplineParams
{
	int uvCountU = 16;
	int uvCountV = 16;
	double gridAngleDeg = 0.0;
	double fitUvSpacingMm = 0.0;
	MeshTrajectoryUvTraceMode traceMode = MeshTrajectoryUvTraceMode::USerpentine;
	/// 与曲面重构 NurbsSurfaceFitting 同源；默认 centripetal 最小二乘 + 指定控制点
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
	DiscretizeParams discretize;
	MeshTrajectoryBsplineParams bspline;
};

struct MeshTrajectoryPolyline
{
	std::vector<RawPathPoint> points;
	bool closed = false;
};

GEOMETRY_ALGORITHM_API bool validateMeshTrajectorySpec(const MeshTrajectorySpec& spec, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool filterSoupByTriangleIndices(const std::vector<float>& triangleSoup,
														const std::vector<int>& triangleIndices,
														std::vector<float>& outSoup,
														std::vector<int>& outOriginalTriangleIndices);

GEOMETRY_ALGORITHM_API bool
intersectPlaneWithTriangleSoup(const std::vector<float>& triangleSoup, const double planeOriginMm[3],
							   const double planeNormalUnit[3], const std::vector<int>* triangleIndexFilter,
							   std::vector<MeshTrajectoryPolyline>& outPolylines, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeMeshTrajectoryPolyline(MeshTrajectoryPolyline& polyline, double stepMm,
															 bool outputTangent, bool outputNormal,
															 const double planeNormalUnit[3]);

GEOMETRY_ALGORITHM_API bool generateMeshTrajectory(const MeshTrajectorySpec& spec,
												   const std::vector<float>& triangleSoup, RawPath& outPath,
												   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool meshTrajectorySpecFromJson(const std::string& jsonUtf8, MeshTrajectorySpec& out,
													   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool meshTrajectorySpecToJson(const MeshTrajectorySpec& spec, std::string& outJsonUtf8);

/// B 样条区域拟合曲面三角化预览（模型坐标 mm，每三角 9 float）
GEOMETRY_ALGORITHM_API bool buildBsplineRegionSurfacePreview(const std::vector<float>& triangleSoup,
															 const MeshTrajectoryRegion& region,
															 const MeshTrajectoryBsplineParams& bspline,
															 std::vector<float>& outTriangleSoupModel,
															 std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_MESHTRAJECTORY_H
