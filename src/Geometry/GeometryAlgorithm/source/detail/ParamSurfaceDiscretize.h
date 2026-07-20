#ifndef GEOMETRYALGORITHM_PARAMSURFACEDISCRETIZE_H
#define GEOMETRYALGORITHM_PARAMSURFACEDISCRETIZE_H

/// @file ParamSurfaceDiscretize.h
/// @brief ParamSurfaceDiscretize 接口

#include "FeatureListDocument.h"

#include <string>

#include <TopoDS_Shape.hxx>

namespace geoalgo
{
bool discretizeFaceParamSurface(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input, RawPath& out,
								std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_PARAMSURFACEDISCRETIZE_H
