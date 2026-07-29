#ifndef GEOMETRYALGORITHM_SKETCHSHELL_H
#define GEOMETRYALGORITHM_SKETCHSHELL_H

/// @file SketchShell.h
/// @brief 选面开壳（OCC MakeThickSolid）

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <string>
#include <vector>

namespace geoalgo
{
GEOMETRY_ALGORITHM_API bool shellFacesToHandle(const ShapeHandle& base, const std::vector<int>& openFaceIndices,
											   double thicknessMm, ShapeHandle& outShape,
											   std::string* errMsg = nullptr);

} // namespace geoalgo

#endif
