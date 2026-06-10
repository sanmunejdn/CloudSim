#pragma once

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>
#include <chrono>

namespace pclalgo
{

struct PerformanceResult
{
    std::string testName;
    double executionTimeMs;
    bool success;
    std::string errorMessage;
};

POINT_CLOUD_ALGORITHM_API bool runPerformanceTest(
    std::vector<PerformanceResult>& results,
    std::string* errMsg = nullptr);

} // namespace pclalgo