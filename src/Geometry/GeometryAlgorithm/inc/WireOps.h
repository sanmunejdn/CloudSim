#ifndef GEOMETRYALGORITHM_WIREOPS_H
#define GEOMETRYALGORITHM_WIREOPS_H

/// @file WireOps.h
/// @brief WireOps 接口

#include "geometry_algorithm_global.h"

#include "Types.h"

#include <string>
#include <vector>

#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API bool fuseWires(const std::vector<TopoDS_Wire>& wires, TopoDS_Wire& outWire,
									  std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool fuseWiresToPolyline(const std::vector<TopoDS_Wire>& wires, const TessellateParams& disc,
												Polyline3d& out, std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_WIREOPS_H
