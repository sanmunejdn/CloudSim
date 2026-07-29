#ifndef GEOMETRYALGORITHM_SKETCHFILLET_H
#define GEOMETRYALGORITHM_SKETCHFILLET_H

/// @file SketchFillet.h
/// @brief 实体边圆角 / 倒角（OCC MakeFillet / MakeChamfer）

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <string>
#include <vector>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API bool filletEdgesToHandle(const ShapeHandle& base, const std::vector<int>& edgeIndices,
												double radiusMm, ShapeHandle& outShape, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool chamferEdgesToHandle(const ShapeHandle& base, const std::vector<int>& edgeIndices,
												 double distanceMm, ShapeHandle& outShape,
												 std::string* errMsg = nullptr);

} // namespace geoalgo

#endif
