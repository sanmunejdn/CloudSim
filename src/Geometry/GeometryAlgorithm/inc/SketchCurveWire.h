#ifndef GEOMETRYALGORITHM_SKETCHCURVEWIRE_H
#define GEOMETRYALGORITHM_SKETCHCURVEWIRE_H

/// @file SketchCurveWire.h
/// @brief 草图曲线段类型（轮廓/路径共用）；OCC wire 构建见 detail/SketchCurveWireOcc.h

#include "geometry_algorithm_global.h"

#include <string>
#include <vector>

namespace geoalgo
{
enum class SketchCurveSegKind
{
	Line = 0,
	Arc = 1,
	SplineThrough = 2,
	/// ax,ay,az=圆心；bx=半径；mx,my,mz=法向（可零则用入参平面法向）
	Circle = 3,
	/// ax,ay,az=中心；bx=长半轴 by=短半轴 bz=面内转角(rad)；m=法向
	Ellipse = 4
};

struct SketchCurveSegment
{
	SketchCurveSegKind kind = SketchCurveSegKind::Line;
	float ax = 0.f, ay = 0.f, az = 0.f;
	float bx = 0.f, by = 0.f, bz = 0.f;
	float mx = 0.f, my = 0.f, mz = 0.f;
};

} // namespace geoalgo

#endif
