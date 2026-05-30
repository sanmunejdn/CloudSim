#pragma once

#include "geometry_algorithm_global.h"
#include "Types.h"

#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>

#include <string>
#include <vector>

namespace geoalgo
{

GEOMETRY_ALGORITHM_API bool fuseWires(
	const std::vector<TopoDS_Wire>& wires,
	TopoDS_Wire& outWire,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool fuseWiresToPolyline(
	const std::vector<TopoDS_Wire>& wires,
	const TessellateParams& disc,
	Polyline3d& out,
	std::string* errMsg = nullptr);

} // namespace geoalgo
