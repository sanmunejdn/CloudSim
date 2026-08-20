#ifndef GEOMETRYALGORITHM_SHAPEQUERY_H
#define GEOMETRYALGORITHM_SHAPEQUERY_H

/// @file ShapeQuery.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Shape 拓扑查询、STEP 路径编排、模型坐标拾取与面/边索引解析

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"
#include "Types.h"

#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
/** Shape 内 Edge 数量（TopExp 遍历顺序，与 shapeEdgeAtIndex 一致） */
GEOMETRY_ALGORITHM_API int shapeEdgeCount(const TopoDS_Shape& shape);

/** ShapeHandle tip 边数（供 Host/AI 无 OCCT 头文件调用） */
GEOMETRY_ALGORITHM_API int shapeHandleEdgeCount(const ShapeHandle& handle);

/** Shape 内 Face 数量 */
GEOMETRY_ALGORITHM_API int shapeFaceCount(const TopoDS_Shape& shape);

/**
 * 按 index 取 Edge
 * @return false：index 越界或 shape 无效
 */
GEOMETRY_ALGORITHM_API bool shapeEdgeAtIndex(const TopoDS_Shape& shape, int index, TopoDS_Edge& outEdge,
											 std::string* errMsg = nullptr);

/**
 * 按 index 取 Face
 * @return false：index 越界
 */
GEOMETRY_ALGORITHM_API bool shapeFaceAtIndex(const TopoDS_Shape& shape, int index, TopoDS_Face& outFace,
											 std::string* errMsg = nullptr);

/** 读 STEP 并离散全部边为折线 */
GEOMETRY_ALGORITHM_API bool discretizeStepEdgesToPolylines(const std::string& pathLocal, const TessellateParams& params,
														   std::vector<Polyline3d>& outPolylines,
														   std::string* errMsg = nullptr);

/** 读 STEP，按 faceIndex 离散单面 mesh */
GEOMETRY_ALGORITHM_API bool discretizeStepFaceToMesh(const std::string& pathLocal, int faceIndex,
													 const MeshDiscretizeParams& params, std::vector<float>& soup,
													 MeshDiscretizeReport& report, std::string* errMsg = nullptr);

/** 读 STEP，按边索引求交 */
GEOMETRY_ALGORITHM_API bool intersectStepEdges(const std::string& pathLocal, int edgeIndex1, int edgeIndex2,
											   const IntersectionParams& params, IntersectionResult& result,
											   std::string* errMsg = nullptr);

/** 读 STEP，边-面求交 */
GEOMETRY_ALGORITHM_API bool intersectStepEdgeFace(const std::string& pathLocal, int edgeIndex, int faceIndex,
												  const IntersectionParams& params, IntersectionResult& result,
												  std::string* errMsg = nullptr);

/** 读 STEP，两面求交 */
GEOMETRY_ALGORITHM_API bool intersectStepFaces(const std::string& pathLocal, int faceIndex1, int faceIndex2,
											   const IntersectionParams& params, IntersectionResult& result,
											   std::string* errMsg = nullptr);

/** 读两个 STEP 文件做形体截面 */
GEOMETRY_ALGORITHM_API bool intersectStepFiles(const std::string& pathLocal1, const std::string& pathLocal2,
											   const IntersectionParams& params, IntersectionResult& result,
											   std::string* errMsg = nullptr);

/** 读两个 STEP，B-rep 布尔后 mesh 离散 */
GEOMETRY_ALGORITHM_API bool brepBooleanStepFilesToMesh(const std::string& targetPathLocal,
													   const std::string& toolPathLocal, BrepBooleanOp op,
													   const MeshDiscretizeParams& meshParams,
													   std::vector<float>& outSoup, std::string* errMsg = nullptr);

/** 读 STEP，按 edgeIndices 融合并离散为折线 */
GEOMETRY_ALGORITHM_API bool fuseStepEdgesToPolyline(const std::string& pathLocal, const std::vector<int>& edgeIndices,
													const TessellateParams& disc, Polyline3d& out,
													std::string* errMsg = nullptr);

/** 读 STEP，按 faceIndices 缝合并 mesh 离散 */
GEOMETRY_ALGORITHM_API bool sewStepFacesToMesh(const std::string& pathLocal, const std::vector<int>& faceIndices,
											   double toleranceMm, const MeshDiscretizeParams& meshParams,
											   std::vector<float>& outSoup, std::string* errMsg = nullptr);

/**
 * 模型坐标点 → 面索引（BRepExtrema 最近面）
 * @param toleranceMm 默认 2.0 mm
 * @return false：null shape 或容差内无面
 */
GEOMETRY_ALGORITHM_API bool resolveFaceIndexFromModelPoint(const ShapeHandle& shape, const Point3d& modelPointMm,
														   int& outFaceIndex, double toleranceMm = 2.0,
														   std::string* errMsg = nullptr);

/**
 * 模型坐标线段 AB 中点 → 边索引
 * @param toleranceMm 默认 2.0 mm
 * @return false：容差内无边
 */
GEOMETRY_ALGORITHM_API bool resolveEdgeIndexFromModelPoints(const ShapeHandle& shape, const Point3d& modelPointA,
															const Point3d& modelPointB, int& outEdgeIndex,
															double toleranceMm = 2.0, std::string* errMsg = nullptr);

/** 读 STEP 后 resolveFaceIndexFromModelPoint */
GEOMETRY_ALGORITHM_API bool resolveStepFaceIndexFromModelPoint(const std::string& stepPathUtf8,
															   const Point3d& modelPointMm, int& outFaceIndex,
															   double toleranceMm = 2.0, std::string* errMsg = nullptr);

/** 读 STEP 后 resolveEdgeIndexFromModelPoints */
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

/**
 * 模型坐标射线 → 面索引（IntCurvesFace_ShapeIntersector）
 * @param rayDirUnit 单位方向
 * @return false：射线未命中面
 */
GEOMETRY_ALGORITHM_API bool pickShapeFaceByModelRay(const ShapeHandle& shape, const Point3d& rayOriginMm,
													const Point3d& rayDirUnit, ShapeRayPickResult& out,
													std::string* errMsg = nullptr);

/**
 * 模型坐标射线 → 边索引；优先在命中面边界上搜索
 * @return false：容差内无边
 */
GEOMETRY_ALGORITHM_API bool pickShapeEdgeByModelRay(const ShapeHandle& shape, const Point3d& rayOriginMm,
													const Point3d& rayDirUnit, double toleranceMm,
													ShapeRayPickResult& out, std::string* errMsg = nullptr);

/**
 * 模型坐标命中点 → 边索引（边拾取首选，不经过 IntCurvesFace）
 * @return false：容差内无边
 */
GEOMETRY_ALGORITHM_API bool pickShapeEdgeByModelPoint(const ShapeHandle& shape, const Point3d& queryPointModelMm,
													  double toleranceMm, ShapeRayPickResult& out,
													  std::string* errMsg = nullptr);

/**
 * 校验 faceIndex 在 shape 范围内
 * @return false：越界或无面
 */
GEOMETRY_ALGORITHM_API bool validateShapeFaceIndex(const ShapeHandle& shape, int faceIndex,
												   std::string* errMsg = nullptr);

/**
 * 按面索引抽出所属 Solid，并从装配体中去掉该 Solid
 * 仅一块 Solid 时 outRemaining 为空，outSolid 为整件
 */
GEOMETRY_ALGORITHM_API bool extractSolidByFaceIndex(const ShapeHandle& assembly, int faceIndex, ShapeHandle& outSolid,
													ShapeHandle& outRemaining, std::string* errMsg = nullptr);

/** 校验 edgeIndex */
GEOMETRY_ALGORITHM_API bool validateShapeEdgeIndex(const ShapeHandle& shape, int edgeIndex,
												   std::string* errMsg = nullptr);

/**
 * 每个 face 的边界边 global index 列表
 * 顺序与 shapeEdgeAtIndex / shapeFaceAtIndex 一致
 */
GEOMETRY_ALGORITHM_API bool collectShapeFaceEdgeIndices(const ShapeHandle& shape,
														std::vector<std::vector<int>>& outFaceEdgeIndices,
														std::string* errMsg = nullptr);

/**
 * 按 faceIndex 离散该面全部边界边（含内环）为多条折线
 * 边索引与 shapeEdgeAtIndex 一致；重复边只输出一次
 */
GEOMETRY_ALGORITHM_API bool discretizeShapeFaceEdgesToPolylines(const ShapeHandle& shape, int faceIndex,
																const TessellateParams& params,
																std::vector<Polyline3d>& outPolylines,
																std::string* errMsg = nullptr);

enum class FaceBoundarySegKind
{
	Line = 0,
	Arc,
	Circle,
	Polyline
};

struct FaceBoundarySeg
{
	FaceBoundarySegKind kind = FaceBoundarySegKind::Polyline;
	std::vector<float> xyz;
	double radiusMm = 0.0;
};

/** 面边界按曲线类型提取（圆/弧优先，其它离散为折线） */
GEOMETRY_ALGORITHM_API bool extractShapeFaceBoundarySegments(const ShapeHandle& shape, int faceIndex,
															 const TessellateParams& fallbackTess,
															 std::vector<FaceBoundarySeg>& outSegs,
															 std::string* errMsg = nullptr);

/** 按边长降序取 Top-K 边索引 */
GEOMETRY_ALGORITHM_API bool selectLongestEdgeIndices(const ShapeHandle& shape, int topK,
													 std::vector<int>& outEdgeIndices, std::string* errMsg = nullptr);

/** Z 最大平面外环边（无平面则退化为 longest） */
GEOMETRY_ALGORITHM_API bool selectTopBoundaryEdgeIndices(const ShapeHandle& shape, std::vector<int>& outEdgeIndices,
														 std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API int shapeHandleVertexCount(const ShapeHandle& handle);

/** 按 TopExp 顶点索引取世界坐标 */
GEOMETRY_ALGORITHM_API bool shapeVertexPointAtIndex(const ShapeHandle& handle, int vertexIndex, Point3d& outPointMm,
													std::string* errMsg = nullptr);

/**
 * 模型点 → 最近 TopExp 顶点
 * @return false：容差内无顶点
 */
GEOMETRY_ALGORITHM_API bool pickShapeVertexByModelPoint(const ShapeHandle& shape, const Point3d& queryPointModelMm,
														double toleranceMm, int& outVertexIndex,
														Point3d& outVertexModelMm, std::string* errMsg = nullptr);

/** 边两端点世界坐标（与 shapeEdgeAtIndex 顺序一致） */
GEOMETRY_ALGORITHM_API bool shapeHandleEdgeEndpoints(const ShapeHandle& handle, int edgeIndex, Point3d& outAMm,
													 Point3d& outBMm, std::string* errMsg = nullptr);

/**
 * 按 TShape 跟踪面归属：未见过的面记到 featureId，并刷新 faceIndex→featureId
 * Data 层勿直接碰 OCC，经此 API 维护进程内归属表
 */
GEOMETRY_ALGORITHM_API void mergeFaceOwnershipByTShape(const ShapeHandle& tip, const std::string& featureId,
													   std::unordered_map<std::uintptr_t, std::string>& tshapeOwners,
													   std::unordered_map<int, std::string>& outFaceIndexOwners);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_SHAPEQUERY_H
