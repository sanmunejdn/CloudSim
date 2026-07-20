#ifndef GEOMETRYALGORITHM_SHAPEQUERY_H
#define GEOMETRYALGORITHM_SHAPEQUERY_H

/// @file ShapeQuery.h
/// @brief 模型坐标点 → 面索引（与 shapeFaceAtIndex 遍历顺序一致）

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"
#include "Types.h"

#include <string>
#include <vector>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API int shapeEdgeCount(const TopoDS_Shape& shape);
GEOMETRY_ALGORITHM_API int shapeFaceCount(const TopoDS_Shape& shape);

GEOMETRY_ALGORITHM_API bool shapeEdgeAtIndex(const TopoDS_Shape& shape, int index, TopoDS_Edge& outEdge,
											 std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool shapeFaceAtIndex(const TopoDS_Shape& shape, int index, TopoDS_Face& outFace,
											 std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeStepEdgesToPolylines(const std::string& pathLocal, const TessellateParams& params,
														   std::vector<Polyline3d>& outPolylines,
														   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeStepFaceToMesh(const std::string& pathLocal, int faceIndex,
													 const MeshDiscretizeParams& params, std::vector<float>& soup,
													 MeshDiscretizeReport& report, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectStepEdges(const std::string& pathLocal, int edgeIndex1, int edgeIndex2,
											   const IntersectionParams& params, IntersectionResult& result,
											   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectStepEdgeFace(const std::string& pathLocal, int edgeIndex, int faceIndex,
												  const IntersectionParams& params, IntersectionResult& result,
												  std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectStepFaces(const std::string& pathLocal, int faceIndex1, int faceIndex2,
											   const IntersectionParams& params, IntersectionResult& result,
											   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectStepFiles(const std::string& pathLocal1, const std::string& pathLocal2,
											   const IntersectionParams& params, IntersectionResult& result,
											   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool brepBooleanStepFilesToMesh(const std::string& targetPathLocal,
													   const std::string& toolPathLocal, BrepBooleanOp op,
													   const MeshDiscretizeParams& meshParams,
													   std::vector<float>& outSoup, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool fuseStepEdgesToPolyline(const std::string& pathLocal, const std::vector<int>& edgeIndices,
													const TessellateParams& disc, Polyline3d& out,
													std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool sewStepFacesToMesh(const std::string& pathLocal, const std::vector<int>& faceIndices,
											   double toleranceMm, const MeshDiscretizeParams& meshParams,
											   std::vector<float>& outSoup, std::string* errMsg = nullptr);

/// 模型坐标点 → 面索引（与 shapeFaceAtIndex 遍历顺序一致）
GEOMETRY_ALGORITHM_API bool resolveFaceIndexFromModelPoint(const ShapeHandle& shape, const Point3d& modelPointMm,
														   int& outFaceIndex, double toleranceMm = 2.0,
														   std::string* errMsg = nullptr);

/// 模型坐标线段 → 边索引（查询点取 AB 中点）
GEOMETRY_ALGORITHM_API bool resolveEdgeIndexFromModelPoints(const ShapeHandle& shape, const Point3d& modelPointA,
															const Point3d& modelPointB, int& outEdgeIndex,
															double toleranceMm = 2.0, std::string* errMsg = nullptr);

/// 模型坐标点 → STEP 面索引（与 shapeFaceAtIndex 遍历顺序一致）
GEOMETRY_ALGORITHM_API bool resolveStepFaceIndexFromModelPoint(const std::string& stepPathUtf8,
															   const Point3d& modelPointMm, int& outFaceIndex,
															   double toleranceMm = 2.0, std::string* errMsg = nullptr);

/// 模型坐标线段 → STEP 边索引（查询点取 AB 中点）
GEOMETRY_ALGORITHM_API bool resolveStepEdgeIndexFromModelPoints(const std::string& stepPathUtf8,
																const Point3d& modelPointA, const Point3d& modelPointB,
																int& outEdgeIndex, double toleranceMm = 2.0,
																std::string* errMsg = nullptr);

struct ShapeRayPickResult
{
	bool hit = false;
	int faceIndex = -1;
	int edgeIndex = -1;
	Point3d hitPointModelMm;
	Point3d edgePointModelMm;
};

/// 模型坐标射线 → BREP 面索引（与 shapeFaceAtIndex 顺序一致）
GEOMETRY_ALGORITHM_API bool pickShapeFaceByModelRay(const ShapeHandle& shape, const Point3d& rayOriginMm,
													const Point3d& rayDirUnit, ShapeRayPickResult& out,
													std::string* errMsg = nullptr);

/// 模型坐标射线 → BREP 边索引；优先在命中面边界上搜索
GEOMETRY_ALGORITHM_API bool pickShapeEdgeByModelRay(const ShapeHandle& shape, const Point3d& rayOriginMm,
													const Point3d& rayDirUnit, double toleranceMm,
													ShapeRayPickResult& out, std::string* errMsg = nullptr);

/// 模型坐标命中点 → BREP 边索引（边拾取首选，不经过 IntCurvesFace）
GEOMETRY_ALGORITHM_API bool pickShapeEdgeByModelPoint(const ShapeHandle& shape, const Point3d& queryPointModelMm,
													  double toleranceMm, ShapeRayPickResult& out,
													  std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool validateShapeFaceIndex(const ShapeHandle& shape, int faceIndex,
												   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool validateShapeEdgeIndex(const ShapeHandle& shape, int edgeIndex,
												   std::string* errMsg = nullptr);

/// 每个 face 的边界边 global index 列表（与 shapeEdgeAtIndex 顺序一致）
GEOMETRY_ALGORITHM_API bool collectShapeFaceEdgeIndices(const ShapeHandle& shape,
														std::vector<std::vector<int>>& outFaceEdgeIndices,
														std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_SHAPEQUERY_H
