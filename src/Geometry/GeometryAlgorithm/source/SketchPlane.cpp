/// @file SketchPlane.cpp

#include "SketchPlane.h"

#include "ShapeQuery.h"
#include "detail/OccIncludes.h"

#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <gp_Ax3.hxx>
#include <gp_Pln.hxx>

namespace geoalgo
{
bool queryPlanarFaceSketchPlane(const ShapeHandle& shape, int faceIndex, SketchPlaneMm& out, std::string* errMsg)
{
	out = {};
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		if (errMsg)
			*errMsg = "null shape";
		return false;
	}
	TopoDS_Face face;
	if (!shapeFaceAtIndex(native, faceIndex, face, errMsg))
		return false;
	BRepAdaptor_Surface surf(face, Standard_True);
	if (surf.GetType() != GeomAbs_Plane)
	{
		if (errMsg)
			*errMsg = "Face is not planar";
		return false;
	}
	const gp_Ax3 ax = surf.Plane().Position();
	const gp_Pnt o = ax.Location();
	const gp_Dir x = ax.XDirection();
	const gp_Dir y = ax.YDirection();
	const gp_Dir n = ax.Direction();
	out.ox = o.X();
	out.oy = o.Y();
	out.oz = o.Z();
	out.xx = x.X();
	out.xy = x.Y();
	out.xz = x.Z();
	out.yx = y.X();
	out.yy = y.Y();
	out.yz = y.Z();
	out.nx = n.X();
	out.ny = n.Y();
	out.nz = n.Z();
	out.planar = true;
	return true;
}
} // namespace geoalgo
