#pragma once

#include "point_cloud_algorithm_global.h"

#include <string>
#include <vector>

namespace pclalgo
{

POINT_CLOUD_ALGORITHM_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace pclalgo
