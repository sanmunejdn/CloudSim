/// @file MeshBoolean.cpp
/// @brief 网格布尔

#include "pch.h"

#include "MeshBoolean.h"

#include <GeoMeshBoolean.h>

// int 强转桥接两个模块的枚举：任一表错位即静默错算，编译期钉住顺序
static_assert(static_cast<int>(MeshBooleanOp::Difference) == static_cast<int>(geoalgo::MeshBooleanOp::Difference) &&
				  static_cast<int>(MeshBooleanOp::Union) == static_cast<int>(geoalgo::MeshBooleanOp::Union) &&
				  static_cast<int>(MeshBooleanOp::Intersection) == static_cast<int>(geoalgo::MeshBooleanOp::Intersection),
			  "MeshBooleanOp enum order mismatch between GeometryServices and GeometryAlgorithm");

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
