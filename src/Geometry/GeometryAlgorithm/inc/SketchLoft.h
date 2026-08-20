#ifndef GEOMETRYALGORITHM_SKETCHLOFT_H
#define GEOMETRYALGORITHM_SKETCHLOFT_H

/// @file SketchLoft.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 两截面放样凸台/切除（OCC ThruSections）

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <string>
#include <vector>

namespace geoalgo
{
enum class SketchLoftMode
{
	Boss = 0,
	Cut
};

struct SketchLoftParams
{
	SketchLoftMode mode = SketchLoftMode::Boss;
	bool solid = true;
};

GEOMETRY_ALGORITHM_API bool sketchLoftPolylinesToHandle(const std::vector<float>& profileAXyzMm,
														const std::vector<float>& profileBXyzMm,
														const SketchLoftParams& params, const ShapeHandle* baseOrNull,
														ShapeHandle& outShape, std::string* errMsg = nullptr);

} // namespace geoalgo

#endif
