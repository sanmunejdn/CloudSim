#ifndef GEOMETRYALGORITHM_SELFTEST_H
#define GEOMETRYALGORITHM_SELFTEST_H

/// @file SelfTest.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
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
