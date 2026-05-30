#include "detail/OccIncludes.h"

#include "MeshDiscretize.h"
#include "ShellOps.h"

namespace geoalgo
{

bool sewFaces(
	const std::vector<TopoDS_Face>& faces,
	const double toleranceMm,
	TopoDS_Shape& outShape,
	std::string* errMsg)
{
	if (faces.empty())
	{
		if (errMsg)
		{
			*errMsg = "no faces to sew";
		}
		return false;
	}
	BRepBuilderAPI_Sewing sewing(toleranceMm);
	for (const TopoDS_Face& face : faces)
	{
		if (!face.IsNull())
		{
			sewing.Add(face);
		}
	}
	sewing.Perform();
	outShape = sewing.SewedShape();
	if (outShape.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "sewing produced empty shape";
		}
		return false;
	}
	return true;
}

bool sewFacesToMesh(
	const std::vector<TopoDS_Face>& faces,
	const double toleranceMm,
	const MeshDiscretizeParams& meshParams,
	std::vector<float>& outSoup,
	std::string* errMsg)
{
	TopoDS_Shape sewn;
	if (!sewFaces(faces, toleranceMm, sewn, errMsg))
	{
		return false;
	}
	MeshDiscretizeReport report;
	return discretizeShapeToMesh(sewn, meshParams, outSoup, report, errMsg);
}

} // namespace geoalgo
