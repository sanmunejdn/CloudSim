/// @file MeshBoolean.cpp
/// @brief MeshBoolean 实现

#include "pch.h"

#include "MeshBoolean.h"

#include <GeoMeshBoolean.h>

namespace MeshBoolean
{
bool compute(const std::vector<float>& targetSoup, const std::vector<float>& toolSoup, const MeshBooleanOp op,
			 std::vector<float>& outSoup, std::string* errMsg)
{
	const auto geoOp = static_cast<geoalgo::MeshBooleanOp>(static_cast<int>(op));
	return geoalgo::meshBooleanCompute(targetSoup, toolSoup, geoOp, outSoup, errMsg);
}

bool runSelfTest(std::string* errMsg)
{
	return geoalgo::meshBooleanRunSelfTest(errMsg);
}

} // namespace MeshBoolean
