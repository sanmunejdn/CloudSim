#pragma once

#include "geometry_algorithm_global.h"
#include "Types.h"

#include <TopoDS_Shape.hxx>

#include <string>
#include <vector>

namespace geoalgo
{

GEOMETRY_ALGORITHM_API bool brepBooleanToShape(
	const TopoDS_Shape& target,
	const TopoDS_Shape& tool,
	BrepBooleanOp op,
	TopoDS_Shape& outShape,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool brepBooleanToMesh(
	const TopoDS_Shape& target,
	const TopoDS_Shape& tool,
	BrepBooleanOp op,
	const MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg = nullptr);

} // namespace geoalgo
