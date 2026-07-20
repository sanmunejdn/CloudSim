#ifndef GEOMETRYALGORITHM_SELFTEST_H
#define GEOMETRYALGORITHM_SELFTEST_H

/// @file SelfTest.h
/// @brief SelfTest 接口

#include "geometry_algorithm_global.h"

#include <string>
#include <vector>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_SELFTEST_H
