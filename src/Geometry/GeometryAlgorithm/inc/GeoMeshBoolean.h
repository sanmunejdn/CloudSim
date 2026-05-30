#pragma once

#include "geometry_algorithm_global.h"
#include "Types.h"

#include <string>
#include <vector>

namespace geoalgo
{

GEOMETRY_ALGORITHM_API bool meshBooleanCompute(
	const std::vector<float>& targetSoup,
	const std::vector<float>& toolSoup,
	MeshBooleanOp op,
	std::vector<float>& outSoup,
	std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool meshBooleanRunSelfTest(std::string* errMsg = nullptr);

} // namespace geoalgo
