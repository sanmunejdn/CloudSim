/// @file PrimitiveBrep.cpp
/// @brief OCCT 基本体造型（与三角 soup 同为原点居中）

#include "PrimitiveBrep.h"

#include "detail/OccIncludes.h"

#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCone.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <BRepPrimAPI_MakeSphere.hxx>
#include <gp_Ax2.hxx>
#include <algorithm>

namespace geoalgo
{
namespace
{
double positive(double v, double fallback)
{
	return v > 1e-9 ? v : fallback;
}
} // namespace

ShapeHandle makePrimitiveShape(const PrimitiveBrepParams& params)
{
	TopoDS_Shape shape;
	switch (params.kind)
	{
	case PrimitiveBrepKind::Box:
	{
		const double lx = positive(params.lengthMm, 100.0);
		const double ly = positive(params.widthMm, 50.0);
		const double lz = positive(params.heightMm, 100.0);
		shape = BRepPrimAPI_MakeBox(gp_Pnt(-lx * 0.5, -ly * 0.5, -lz * 0.5), lx, ly, lz).Shape();
		break;
	}
	case PrimitiveBrepKind::Cylinder:
	{
		const double r = positive(params.radiusMm, 30.0);
		const double h = positive(params.heightMm, 100.0);
		const gp_Ax2 axis(gp_Pnt(0.0, 0.0, -h * 0.5), gp::DZ());
		shape = BRepPrimAPI_MakeCylinder(axis, r, h).Shape();
		break;
	}
	case PrimitiveBrepKind::Cone:
	{
		const double rb = positive(params.radiusMm, 30.0);
		const double rt = std::max(0.0, params.radiusTopMm);
		const double h = positive(params.heightMm, 100.0);
		const gp_Ax2 axis(gp_Pnt(0.0, 0.0, -h * 0.5), gp::DZ());
		shape = BRepPrimAPI_MakeCone(axis, rb, rt, h).Shape();
		break;
	}
	case PrimitiveBrepKind::Sphere:
	{
		const double r = positive(params.radiusMm, 30.0);
		shape = BRepPrimAPI_MakeSphere(gp_Pnt(0.0, 0.0, 0.0), r).Shape();
		break;
	}
	}
	if (shape.IsNull())
		return {};
	return ShapeHandleAccess::fromNativeShape(&shape);
}
} // namespace geoalgo
