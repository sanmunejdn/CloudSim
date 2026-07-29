#ifndef GEOMETRYALGORITHM_SKETCHPATTERN_H
#define GEOMETRYALGORITHM_SKETCHPATTERN_H

/// @file SketchPattern.h
/// @brief 体线性阵列 / 相对平面镜像（MVP：作用于 tip 实体）

#include "geometry_algorithm_global.h"
#include "ShapeHandle.h"

#include <string>

namespace geoalgo
{
struct SketchLinearPatternParams
{
	int count = 2;
	double dxMm = 10.0;
	double dyMm = 0.0;
	double dzMm = 0.0;
};

struct SketchMirror3dParams
{
	double ox = 0, oy = 0, oz = 0;
	double nx = 1, ny = 0, nz = 0;
	/// true：保留原件并 Fuse 镜像；false：仅输出镜像体
	bool keepOriginal = true;
};

GEOMETRY_ALGORITHM_API bool linearPatternBodyToHandle(const ShapeHandle& seed, const SketchLinearPatternParams& params,
													  ShapeHandle& outShape, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool mirrorBodyToHandle(const ShapeHandle& seed, const SketchMirror3dParams& params,
											   ShapeHandle& outShape, std::string* errMsg = nullptr);

} // namespace geoalgo

#endif
