#pragma once

#include "geometry_algorithm_global.h"

#include <string>

class TopoDS_Shape;

namespace geoalgo
{

/// STEP 读入；path 为 Qt encodeName 窄字节
GEOMETRY_ALGORITHM_API bool readStepShape(const std::string& pathLocal, TopoDS_Shape& outShape, std::string* errMsg);

} // namespace geoalgo
