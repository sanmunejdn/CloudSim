/// @file CsgkBackend.cpp
/// @brief CloudSimGeomKernel 适配实现

#include "CsgkBackend.h"

#include "ShapeHandle.h"

#ifdef CLOUDSIM_USE_CSGK

#include <csgk/body_handle.h>
#include <csgk/discretize.h>
#include <csgk/io_native.h>
#include <csgk/tessellate.h>
#include <csgk/topology_query.h>

namespace geoalgo
{
bool readCsgkNativeFile(const std::string& pathLocal, ShapeHandle& outShape, std::string* errMsg)
{
	const auto loaded = csgk::readNative(pathLocal);
	if(!loaded.ok())
	{
		if(errMsg)
			*errMsg = loaded.message.empty() ? "csgk readNative failed" : loaded.message;
		return false;
	}
	outShape = ShapeHandleAccess::fromCsgkBody(loaded.value);
	return !outShape.isNull();
}

bool discretizeCsgkShapeToSoup(const ShapeHandle& shape, const TessellateParams& params, std::vector<float>& outSoup,
							   std::string* errMsg)
{
	csgk::BodyHandle body;
	if(!ShapeHandleAccess::tryGetCsgkBody(shape, body))
	{
		if(errMsg)
			*errMsg = "not a csgk-backed shape";
		return false;
	}
	csgk::TessellateParams tp;
	tp.linearDeflectionMm = params.linearDeflectionMm;
	tp.angularDeflectionDeg = params.angularDeflectionDeg;
	tp.linearDeflectionRelative = params.linearDeflectionRelative;
	if(!csgk::discretizeToSoup(body, tp, outSoup, nullptr))
	{
		if(errMsg)
			*errMsg = "csgk discretize failed";
		return false;
	}
	return !outSoup.empty();
}

int csgkShapeFaceCount(const ShapeHandle& shape)
{
	csgk::BodyHandle body;
	if(!ShapeHandleAccess::tryGetCsgkBody(shape, body))
		return 0;
	return csgk::faceCount(body);
}

int csgkShapeEdgeCount(const ShapeHandle& shape)
{
	csgk::BodyHandle body;
	if(!ShapeHandleAccess::tryGetCsgkBody(shape, body))
		return 0;
	return csgk::edgeCount(body);
}

} // namespace geoalgo

#endif // CLOUDSIM_USE_CSGK
