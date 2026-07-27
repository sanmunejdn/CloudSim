#ifndef GEOMETRYALGORITHM_SKETCHPLANE_H
#define GEOMETRYALGORITHM_SKETCHPLANE_H

/// @file SketchPlane.h
/// @brief 从 B-rep 面提取草图平面（世界 mm），无暴露 OCCT 类型

#include "geometry_algorithm_global.h"

#include "ShapeHandle.h"

#include <string>

namespace geoalgo
{
struct SketchPlaneMm
{
	double ox = 0, oy = 0, oz = 0;
	double xx = 1, xy = 0, xz = 0;
	double yx = 0, yy = 1, yz = 0;
	double nx = 0, ny = 0, nz = 1;
	bool planar = false;
};

GEOMETRY_ALGORITHM_API bool queryPlanarFaceSketchPlane(const ShapeHandle& shape, int faceIndex, SketchPlaneMm& out,
													   std::string* errMsg = nullptr);

} // namespace geoalgo

#endif
