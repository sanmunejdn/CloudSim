#include "detail/OccIncludes.h"

#include "Discretize.h"
#include "WireOps.h"

namespace geoalgo
{

bool fuseWires(const std::vector<TopoDS_Wire>& wires, TopoDS_Wire& outWire, std::string* errMsg)
{
	if (wires.empty())
	{
		if (errMsg)
		{
			*errMsg = "no wires to fuse";
		}
		return false;
	}
	BRepBuilderAPI_MakeWire builder;
	for (const TopoDS_Wire& wire : wires)
	{
		if (wire.IsNull())
		{
			continue;
		}
		builder.Add(wire);
	}
	if (!builder.IsDone())
	{
		if (errMsg)
		{
			*errMsg = "MakeWire failed";
		}
		return false;
	}
	outWire = builder.Wire();
	return !outWire.IsNull();
}

bool fuseWiresToPolyline(
	const std::vector<TopoDS_Wire>& wires,
	const TessellateParams& disc,
	Polyline3d& out,
	std::string* errMsg)
{
	TopoDS_Wire fused;
	if (!fuseWires(wires, fused, errMsg))
	{
		return false;
	}
	return discretizeWire(fused, disc, out, errMsg);
}

} // namespace geoalgo
