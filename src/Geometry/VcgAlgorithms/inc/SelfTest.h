#ifndef VCGALGORITHMS_SELFTEST_H
#define VCGALGORITHMS_SELFTEST_H

/// @file SelfTest.h
/// @brief SelfTest 接口

#include "vcg_algorithms_global.h"

#include <string>
#include <vector>

namespace vcgalgo
{
// 运行所有自检，返回 false 时 failures 非空
VCg_ALGORITHMS_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace vcgalgo

#endif // VCGALGORITHMS_SELFTEST_H
