#ifndef GEOMETRYENGINE_SELFTEST_H
#define GEOMETRYENGINE_SELFTEST_H

/// @file SelfTest.h
/// @brief SelfTest 接口

#include "geometry_engine_global.h"

#include <string>
#include <vector>

namespace engine
{
GEOMETRY_ENGINE_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace engine

#endif // GEOMETRYENGINE_SELFTEST_H
