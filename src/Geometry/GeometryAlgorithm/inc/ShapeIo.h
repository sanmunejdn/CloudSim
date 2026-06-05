#pragma once

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <string>

class TopoDS_Shape;

namespace geoalgo
{

/// STEP 读入；path 为 Qt encodeName 窄字节
GEOMETRY_ALGORITHM_API bool readStepShape(const std::string& pathLocal, TopoDS_Shape& outShape, std::string* errMsg);
GEOMETRY_ALGORITHM_API bool readStepIntoHandle(const std::string& pathLocal, ShapeHandle& outShape, std::string* errMsg);

GEOMETRY_ALGORITHM_API bool readBrepFile(const std::string& pathLocal, ShapeHandle& outShape, std::string* errMsg);
GEOMETRY_ALGORITHM_API bool writeBrepFile(const std::string& pathLocal, const ShapeHandle& shape, std::string* errMsg);

} // namespace geoalgo
