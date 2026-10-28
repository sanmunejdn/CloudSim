/// @file IFeatureDiscretizer.cpp
/// @brief IFeatureDiscretizer 实现

#include "IFeatureDiscretizer.h"

#include "FeatureDiscretizerRegistry.h"

#include <TopoDS_Shape.hxx>

namespace geoalgo
{
namespace
{
bool slotToTopo(const ShapeHandle& handle, TopoDS_Shape& out)
{
	return ShapeHandleAccess::nativeShape(handle, &out) && !out.IsNull();
}
} // namespace

bool IFeatureDiscretizer::validate(const FeatureDiscretizeInput& input, std::string* errMsg) const
{
	if (input.strategyId != strategyId())
	{
		if (errMsg)
		{
			*errMsg = "strategyId mismatch";
		}
		return false;
	}
	switch (affinity())
	{
	case GeometryAffinity::Line:
		if (!geometryHasKind(input.geometry, GeometryRefKind::Edge))
		{
			if (errMsg)
			{
				*errMsg = "Line strategy requires Edge refs";
			}
			return false;
		}
		break;
	case GeometryAffinity::Face:
		if (!geometryHasKind(input.geometry, GeometryRefKind::Face))
		{
			if (errMsg)
			{
				*errMsg = "Face strategy requires Face refs";
			}
			return false;
		}
		break;
	case GeometryAffinity::Any:
		break;
	}
	return true;
}

bool IFeatureDiscretizer::discretizeEx(const FeatureDiscretizeContext& ctx, RawPath& out, std::string* errMsg) const
{
	TopoDS_Shape shape;
	if (!slotToTopo(ctx.primaryShapeHandle, shape))
	{
		for (const ShapeSlot& slot : ctx.shapeSlots)
		{
			if (slotToTopo(slot.shapeHandle, shape))
			{
				break;
			}
		}
	}
	if (shape.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "FeatureDiscretizeContext has no primary shape";
		}
		return false;
	}
	return discretize(shape, ctx.input, out, errMsg);
}

} // namespace geoalgo
