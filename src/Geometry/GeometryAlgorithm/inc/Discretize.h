#ifndef GEOMETRYALGORITHM_DISCRETIZE_H
#define GEOMETRYALGORITHM_DISCRETIZE_H

/// @file Discretize.h
/// @brief Edge/Wire/Shape 折线与三角 soup 离散；STEP 单件/层级

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"
#include "Types.h"

#include <string>
#include <vector>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>

namespace geoalgo
{
/** 按 Coarse/Medium/Fine 写入 tessellate 线性/角偏差（Custom 不改） */
GEOMETRY_ALGORITHM_API void applyQualityPreset(MeshDiscretizeParams& params);

/**
 * 单条 Edge → 折线（mm，3N float）
 * @param params linearDeflectionMm 默认 0.01；angularDeflectionDeg 默认 0.5°
 * @return false：OCCT 离散失败或折线为空
 */
GEOMETRY_ALGORITHM_API bool discretizeEdge(const TopoDS_Edge& edge, const TessellateParams& params, Polyline3d& out,
										   std::string* errMsg = nullptr);

/**
 * Wire 内各 Edge 顺序拼接为一条折线
 * @return false：wire 无效或离散为空
 */
GEOMETRY_ALGORITHM_API bool discretizeWire(const TopoDS_Wire& wire, const TessellateParams& params, Polyline3d& out,
										   std::string* errMsg = nullptr);

/** Shape 内全部 Edge 逐条离散，输出多条折线 */
GEOMETRY_ALGORITHM_API bool discretizeShapeEdges(const TopoDS_Shape& shape, const TessellateParams& params,
												 std::vector<Polyline3d>& out, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeShapeEdges(const ShapeHandle& shape, const TessellateParams& params,
												 std::vector<Polyline3d>& out, std::string* errMsg = nullptr);

/**
 * 按 faceIndex（与 shapeFaceAtIndex 顺序一致）离散单面为三角 soup
 * @return false：索引越界、null shape 或面三角为空
 */
GEOMETRY_ALGORITHM_API bool discretizeShapeFaceByIndex(const ShapeHandle& shape, int faceIndex,
													   const TessellateParams& params, std::vector<float>& soup,
													   std::string* errMsg = nullptr);

/**
 * 按 edgeIndex 离散单条边
 * @return false：索引越界或离散为空
 */
GEOMETRY_ALGORITHM_API bool discretizeShapeEdgeByIndex(const ShapeHandle& shape, int edgeIndex,
													   const TessellateParams& params, Polyline3d& out,
													   std::string* errMsg = nullptr);

/**
 * 单 Face → 9T float 三角 soup（先 BRepMesh 再提取）
 * @return false：面三角化 empty（"face triangulation empty"）
 */
GEOMETRY_ALGORITHM_API bool discretizeFaceToSoup(const TopoDS_Face& face, const TessellateParams& params,
												 std::vector<float>& soup, std::string* errMsg = nullptr);

/**
 * 整件 Shape 合并三角 soup
 * @return false：null shape 或 soup 为空（"shape triangulation empty"）
 */
GEOMETRY_ALGORITHM_API bool discretizeShapeToSoup(const TopoDS_Shape& shape, const TessellateParams& params,
												  std::vector<float>& soup, std::string* errMsg = nullptr);

/**
 * 整件 mesh 一次（InParallel），再按 shapeFaceAtIndex 顺序逐面提取
 * @param outTriangleFaceIndex 每三角对应面索引
 * @param outFaceSoups 可选，每面局部 soup
 * @return false：null shape、无面或 mesh 为空
 */
GEOMETRY_ALGORITHM_API bool discretizeShapeToSoupPerFace(const TopoDS_Shape& shape, const TessellateParams& params,
														 std::vector<float>& outSoup,
														 std::vector<int>& outTriangleFaceIndex,
														 std::vector<std::vector<float>>* outFaceSoups = nullptr,
														 std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool discretizeShapeToSoupPerFace(const ShapeHandle& shape, const TessellateParams& params,
														 std::vector<float>& outSoup,
														 std::vector<int>& outTriangleFaceIndex,
														 std::vector<std::vector<float>>* outFaceSoups = nullptr,
														 std::string* errMsg = nullptr);

/**
 * 读 STEP 并整件离散为 soup
 * @param pathLocal Qt encodeName 窄字节路径
 * @return false：STEP 读入失败或离散为空
 */
GEOMETRY_ALGORITHM_API bool tessellateStepFile(const std::string& pathLocal, const TessellateParams& params,
											   std::vector<float>& soup, std::string* errMsg = nullptr);

/**
 * 读 STEP，按装配层级逐零件离散（每零件含 triangleSoup）
 * @return false：读入失败或层级无 mesh（"shape hierarchy triangulation produced no mesh parts"）
 */
GEOMETRY_ALGORITHM_API bool tessellateStepHierarchy(const std::string& pathLocal, const TessellateParams& params,
													std::vector<MeshHierarchyPart>& outParts,
													std::string* errMsg = nullptr);

/** 已有 Shape 的层级离散（带 tessellation） */
GEOMETRY_ALGORITHM_API bool collectShapeHierarchy(const TopoDS_Shape& shape, const TessellateParams& params,
												  std::vector<MeshHierarchyPart>& outParts,
												  std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool collectShapeHierarchy(const ShapeHandle& shape, const TessellateParams& params,
												  std::vector<MeshHierarchyPart>& outParts,
												  std::string* errMsg = nullptr);

/**
 * 仅遍历 OCCT 装配树输出路径/显示名，不填充 triangleSoup
 * 供 BREP 装配导入；各零件共享同一 assembly ShapeHandle
 */
GEOMETRY_ALGORITHM_API bool collectShapeHierarchyTopology(const ShapeHandle& shape,
														  std::vector<MeshHierarchyPart>& outParts,
														  std::string* errMsg = nullptr);

/**
 * BRepMesh_IncrementalMesh 整件三角化（不写 soup，供后续按面提取）
 * @return false：null shape
 */
GEOMETRY_ALGORITHM_API bool meshShapeIncremental(const TopoDS_Shape& shape, const TessellateParams& params,
												 std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_DISCRETIZE_H
