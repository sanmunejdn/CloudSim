#ifndef GEOMETRYALGORITHM_BREPBOOLEAN_H
#define GEOMETRYALGORITHM_BREPBOOLEAN_H

/// @file BrepBoolean.h
/// @brief BrepBoolean 接口

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <string>
#include <vector>

#include <TopoDS_Shape.hxx>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API bool brepBooleanToShape(const TopoDS_Shape& target, const TopoDS_Shape& tool, BrepBooleanOp op,
											   TopoDS_Shape& outShape, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool brepBooleanToMesh(const TopoDS_Shape& target, const TopoDS_Shape& tool, BrepBooleanOp op,
											  const MeshDiscretizeParams& meshParams, std::vector<float>& outSoup,
											  std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_BREPBOOLEAN_H
