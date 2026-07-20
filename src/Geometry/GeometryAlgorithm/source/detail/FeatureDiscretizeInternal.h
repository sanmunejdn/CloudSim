#ifndef GEOMETRYALGORITHM_FEATUREDISCRETIZEINTERNAL_H
#define GEOMETRYALGORITHM_FEATUREDISCRETIZEINTERNAL_H

/// @file FeatureDiscretizeInternal.h
/// @brief FeatureDiscretizeInternal 接口

#include "FeatureListDocument.h"
#include "ShapeHandle.h"

#include <string>

#include <TopoDS_Shape.hxx>

namespace geoalgo
{
bool discretizeFeatureListInternal(const FeatureListDocument& doc, const TopoDS_Shape& shape, RawPath& out,
								   std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FEATUREDISCRETIZEINTERNAL_H
