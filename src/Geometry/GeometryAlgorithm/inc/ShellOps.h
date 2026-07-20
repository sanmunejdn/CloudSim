#ifndef GEOMETRYALGORITHM_SHELLOPS_H
#define GEOMETRYALGORITHM_SHELLOPS_H

/// @file ShellOps.h
/// @brief ShellOps 接口

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <string>
#include <vector>

#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API bool sewFaces(const std::vector<TopoDS_Face>& faces, double toleranceMm, TopoDS_Shape& outShape,
									 std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool sewFacesToMesh(const std::vector<TopoDS_Face>& faces, double toleranceMm,
										   const MeshDiscretizeParams& meshParams, std::vector<float>& outSoup,
										   std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_SHELLOPS_H
