#ifndef GEOMETRYALGORITHM_SKETCHPATTERN_H
#define GEOMETRYALGORITHM_SKETCHPATTERN_H

/// @file SketchPattern.h
/// @brief 体线性/圆周阵列与相对平面镜像

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

struct SketchCircularPatternParams
{
	int count = 2;
	double angleDeg = 360.0;
	double axisOx = 0, axisOy = 0, axisOz = 0;
	double axisDx = 0, axisDy = 0, axisDz = 1;
};

struct SketchMirror3dParams
{
	double ox = 0, oy = 0, oz = 0;
	double nx = 1, ny = 0, nz = 0;
	/// true：保留原件并 Fuse 镜像；false：仅输出镜像体
	bool keepOriginal = true;
};

/**
 * 线性阵列
 * @param fuseOnto 非空时：把 seed 的副本(1..count-1) fuse 到现有 tip（特征级阵列，保留中间特征）
 *                 空：以 seed 为原件做完整阵列
 */
GEOMETRY_ALGORITHM_API bool linearPatternBodyToHandle(const ShapeHandle& seed, const SketchLinearPatternParams& params,
													  ShapeHandle& outShape, std::string* errMsg = nullptr,
													  const ShapeHandle* fuseOnto = nullptr);

GEOMETRY_ALGORITHM_API bool circularPatternBodyToHandle(const ShapeHandle& seed,
														const SketchCircularPatternParams& params,
														ShapeHandle& outShape, std::string* errMsg = nullptr,
														const ShapeHandle* fuseOnto = nullptr);

/// 阵列源特征贡献体：after CUT before；before 空则返回 after；Cut 失败回退 after
GEOMETRY_ALGORITHM_API bool featureContributionSeed(const ShapeHandle& tipAfter, const ShapeHandle& tipBefore,
													ShapeHandle& outSeed, std::string* errMsg = nullptr);

GEOMETRY_ALGORITHM_API bool mirrorBodyToHandle(const ShapeHandle& seed, const SketchMirror3dParams& params,
											   ShapeHandle& outShape, std::string* errMsg = nullptr);

} // namespace geoalgo

#endif
