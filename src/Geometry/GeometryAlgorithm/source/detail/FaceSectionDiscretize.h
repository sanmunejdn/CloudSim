#pragma once

#include "FeatureListDocument.h"

#include <TopoDS_Shape.hxx>

#include <string>

namespace geoalgo
{

bool discretizeFaceSectionGrid(
	const TopoDS_Shape& shape,
	const FeatureDiscretizeInput& input,
	RawPath& out,
	std::string* errMsg = nullptr);

} // namespace geoalgo
