#pragma once

#include "geometry_engine_global.h"

#include <string>
#include <vector>

namespace engine
{

GEOMETRY_ENGINE_API bool runSelfTest(std::vector<std::string>& failures);

} // namespace engine
