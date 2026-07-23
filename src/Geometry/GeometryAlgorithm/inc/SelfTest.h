#ifndef GEOMETRYALGORITHM_SELFTEST_H
#define GEOMETRYALGORITHM_SELFTEST_H

/// @file SelfTest.h
/// @brief 模块内置自检（离散/求交/布尔/轨迹/管状/曲面重构等）

#include "geometry_algorithm_global.h"

#include <string>
#include <vector>

namespace geoalgo
{
/**
 * 运行全部自检用例
 * @param failures 失败用例名列表
 * @return 全部通过为 true
 */
GEOMETRY_ALGORITHM_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_SELFTEST_H
