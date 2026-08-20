#ifndef GEOMETRYALGORITHM_INTERSECTION_H
#define GEOMETRYALGORITHM_INTERSECTION_H

/// @file Intersection.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief OCCT 线线、线面、面面、形体截面求交

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <string>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
/**
 * 两 Edge 求交
 * @param params toleranceMm 默认 1e-3 mm；discretizeCurves 为 true 时曲线段写入 result.curves
 * @return false：OCCT 求交失败（"edge-edge intersection failed"）
 */
GEOMETRY_ALGORITHM_API bool intersectEdges(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2,
										   const IntersectionParams& params, IntersectionResult& result,
										   std::string* errMsg = nullptr);

/**
 * Edge 与 Face 截面
 * @return false：BRepAlgoAPI_Section 失败
 */
GEOMETRY_ALGORITHM_API bool intersectEdgeFace(const TopoDS_Edge& edge, const TopoDS_Face& face,
											  const IntersectionParams& params, IntersectionResult& result,
											  std::string* errMsg = nullptr);

/**
 * 两 Face 求交曲线/点
 * @return false：face-face intersection failed
 */
GEOMETRY_ALGORITHM_API bool intersectFaces(const TopoDS_Face& face1, const TopoDS_Face& face2,
										   const IntersectionParams& params, IntersectionResult& result,
										   std::string* errMsg = nullptr);

/**
 * 两 Shape 布尔截面（BRepAlgoAPI_Section）
 * @return false：Section 失败
 */
GEOMETRY_ALGORITHM_API bool intersectShapes(const TopoDS_Shape& shape1, const TopoDS_Shape& shape2,
											const IntersectionParams& params, IntersectionResult& result,
											std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_INTERSECTION_H
