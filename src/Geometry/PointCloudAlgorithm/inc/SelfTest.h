#ifndef POINTCLOUDALGORITHM_SELFTEST_H
#define POINTCLOUDALGORITHM_SELFTEST_H

/// @file SelfTest.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief SelfTest 接口

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

namespace pclalgo
{
POINT_CLOUD_ALGORITHM_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_SELFTEST_H
