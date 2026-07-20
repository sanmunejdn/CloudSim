#ifndef GEOMETRYALGORITHM_FACESECTIONDISCRETIZE_H
#define GEOMETRYALGORITHM_FACESECTIONDISCRETIZE_H

/// @file FaceSectionDiscretize.h
/// @brief FaceSectionDiscretize 接口

#include "FeatureListDocument.h"

#include <string>

#include <TopoDS_Shape.hxx>

namespace geoalgo
{
bool discretizeFaceSectionGrid(const TopoDS_Shape& shape, const FeatureDiscretizeInput& input, RawPath& out,
							   std::string* errMsg = nullptr);

} // namespace geoalgo

#endif // GEOMETRYALGORITHM_FACESECTIONDISCRETIZE_H
