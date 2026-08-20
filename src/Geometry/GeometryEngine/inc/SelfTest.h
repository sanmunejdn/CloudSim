#ifndef GEOMETRYENGINE_SELFTEST_H
#define GEOMETRYENGINE_SELFTEST_H

/// @file SelfTest.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief SelfTest 接口

#include "geometry_engine_global.h"

#include <string>
#include <vector>

namespace engine
{
GEOMETRY_ENGINE_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace engine

#endif // GEOMETRYENGINE_SELFTEST_H
