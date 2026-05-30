#include "detail/OccIncludes.h"

#include "BrepBoolean.h"
#include "MeshDiscretize.h"

namespace geoalgo
{

bool brepBooleanToShape(
	const TopoDS_Shape& target,
	const TopoDS_Shape& tool,
	const BrepBooleanOp op,
	TopoDS_Shape& outShape,
	std::string* errMsg)
{
	ShapeFix_Shape fixer(target);
	fixer.Perform();
	const TopoDS_Shape fixedTarget = fixer.Shape();

	BRepAlgoAPI_BooleanOperation* algo = nullptr;
	BRepAlgoAPI_Fuse fuse(fixedTarget, tool);
	BRepAlgoAPI_Common common(fixedTarget, tool);
	BRepAlgoAPI_Cut cut(fixedTarget, tool);
	switch (op)
	{
	case BrepBooleanOp::Fuse:
		algo = &fuse;
		break;
	case BrepBooleanOp::Common:
		algo = &common;
		break;
	case BrepBooleanOp::Cut:
		algo = &cut;
		break;
	}
	algo->Build();
	if (!algo->IsDone())
	{
		if (errMsg)
		{
			*errMsg = "BRep boolean operation failed";
		}
		return false;
	}
	outShape = algo->Shape();
	if (outShape.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "BRep boolean produced empty shape";
		}
		return false;
	}
	return true;
}

bool brepBooleanToMesh(
	const TopoDS_Shape& target,
	const TopoDS_Shape& tool,
	const BrepBooleanOp op,
	const MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg)
{
	TopoDS_Shape result;
	if (!brepBooleanToShape(target, tool, op, result, errMsg))
	{
		return false;
	}
	MeshDiscretizeReport report;
	return discretizeShapeToMesh(result, meshParams, outSoup, report, errMsg);
}

} // namespace geoalgo
