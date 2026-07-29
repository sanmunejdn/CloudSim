#ifndef GEOMETRYALGORITHM_SKETCHREVOLVE_H
#define GEOMETRYALGORITHM_SKETCHREVOLVE_H

/// @file SketchRevolve.h
/// @brief 闭合轮廓绕轴旋转凸台/切除

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <string>
#include <vector>

namespace geoalgo
{
enum class SketchRevolveMode
{
	Boss = 0,
	Cut
};

struct SketchRevolveParams
{
	SketchRevolveMode mode = SketchRevolveMode::Boss;
	/// 角度（度），默认整周
	double angleDeg = 360.0;
	double axisOx = 0, axisOy = 0, axisOz = 0;
	double axisDx = 0, axisDy = 0, axisDz = 1;
};

GEOMETRY_ALGORITHM_API bool sketchRevolvePolylineToHandle(const std::vector<float>& profilePolylineXyzMm,
														  const SketchRevolveParams& params,
														  const ShapeHandle* baseOrNull, ShapeHandle& outShape,
														  std::string* errMsg = nullptr);

} // namespace geoalgo

#endif
