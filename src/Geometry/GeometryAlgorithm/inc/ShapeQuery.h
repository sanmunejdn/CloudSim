#pragma once

#include "geometry_algorithm_global.h"
#include "Types.h"

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

#include <string>
#include <vector>

namespace geoalgo
{

GEOMETRY_ALGORITHM_API int shapeEdgeCount(const TopoDS_Shape& shape);
GEOMETRY_ALGORITHM_API int shapeFaceCount(const TopoDS_Shape& shape);

GEOMETRY_ALGORITHM_API bool shapeEdgeAtIndex(
	const TopoDS_Shape& shape,
	int index,
	TopoDS_Edge& outEdge,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool shapeFaceAtIndex(
	const TopoDS_Shape& shape,
	int index,
	TopoDS_Face& outFace,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeStepEdgesToPolylines(
	const std::string& pathLocal,
	const TessellateParams& params,
	std::vector<Polyline3d>& outPolylines,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeStepFaceToMesh(
	const std::string& pathLocal,
	int faceIndex,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	MeshDiscretizeReport& report,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectStepEdges(
	const std::string& pathLocal,
	int edgeIndex1,
	int edgeIndex2,
	const IntersectionParams& params,
	IntersectionResult& result,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectStepEdgeFace(
	const std::string& pathLocal,
	int edgeIndex,
	int faceIndex,
	const IntersectionParams& params,
	IntersectionResult& result,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectStepFaces(
	const std::string& pathLocal,
	int faceIndex1,
	int faceIndex2,
	const IntersectionParams& params,
	IntersectionResult& result,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectStepFiles(
	const std::string& pathLocal1,
	const std::string& pathLocal2,
	const IntersectionParams& params,
	IntersectionResult& result,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool brepBooleanStepFilesToMesh(
	const std::string& targetPathLocal,
	const std::string& toolPathLocal,
	BrepBooleanOp op,
	const MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool fuseStepEdgesToPolyline(
	const std::string& pathLocal,
	const std::vector<int>& edgeIndices,
	const TessellateParams& disc,
	Polyline3d& out,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool sewStepFacesToMesh(
	const std::string& pathLocal,
	const std::vector<int>& faceIndices,
	double toleranceMm,
	const MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg = nullptr);

/// 模型坐标点 → STEP 面索引（与 shapeFaceAtIndex 遍历顺序一致）
GEOMETRY_ALGORITHM_API bool resolveStepFaceIndexFromModelPoint(
	const std::string& stepPathUtf8,
	const Point3d& modelPointMm,
	int& outFaceIndex,
	double toleranceMm = 2.0,
	std::string* errMsg = nullptr);

/// 模型坐标线段 → STEP 边索引（查询点取 AB 中点）
GEOMETRY_ALGORITHM_API bool resolveStepEdgeIndexFromModelPoints(
	const std::string& stepPathUtf8,
	const Point3d& modelPointA,
	const Point3d& modelPointB,
	int& outEdgeIndex,
	double toleranceMm = 2.0,
	std::string* errMsg = nullptr);

} // namespace geoalgo
