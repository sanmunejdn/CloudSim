#ifndef GEOMETRYALGORITHM_INTERSECTION_H
#define GEOMETRYALGORITHM_INTERSECTION_H

/// @file Intersection.h
/// @brief Intersection 接口

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <string>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API bool intersectEdges(const TopoDS_Edge& edge1, const TopoDS_Edge& edge2,
										   const IntersectionParams& params, IntersectionResult& result,
										   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectEdgeFace(const TopoDS_Edge& edge, const TopoDS_Face& face,
											  const IntersectionParams& params, IntersectionResult& result,
											  std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectFaces(const TopoDS_Face& face1, const TopoDS_Face& face2,
										   const IntersectionParams& params, IntersectionResult& result,
										   std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool intersectShapes(const TopoDS_Shape& shape1, const TopoDS_Shape& shape2,
											const IntersectionParams& params, IntersectionResult& result,
											std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_INTERSECTION_H
