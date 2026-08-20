#ifndef POINTCLOUDALGORITHM_PERFORMANCETEST_H
#define POINTCLOUDALGORITHM_PERFORMANCETEST_H

/// @file PerformanceTest.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief PerformanceTest 接口

#include "point_cloud_algorithm_global.h"

#include <chrono>
#include <string>
#include <vector>

namespace pclalgo
{
struct PerformanceResult
{
	std::string testName;
	double executionTimeMs;
	bool success;
	std::string errorMessage;
};

POINT_CLOUD_ALGORITHM_API bool runPerformanceTest(std::vector<PerformanceResult>& results,
												  std::string* errMsg = nullptr);

} // namespace pclalgo

#endif // POINTCLOUDALGORITHM_PERFORMANCETEST_H
