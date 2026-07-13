#pragma once

#include "FeatureListDocument.h"
#include "ShapeHandle.h"

#include <TopoDS_Shape.hxx>

#include <string>

namespace geoalgo
{

bool discretizeFeatureListInternal(
	const FeatureListDocument& doc,
	const TopoDS_Shape& shape,
	RawPath& out,
	std::string* errMsg = nullptr);

} // namespace geoalgo
