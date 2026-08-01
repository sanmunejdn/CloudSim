/// @file DrawingGeomFactory.cpp
/// @brief TopoDS_Edge → DrawingEntity；HLR 边几何以离散为准

#include "DrawingGeometry.h"

#include "Discretize.h"

#include <BRepAdaptor_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

namespace geoalgo
{
namespace
{

void fillPolylineXyFromPoly(const Polyline3d& poly, DrawingEntity& out)
{
	out.polylineXy.clear();
	out.polylineXy.reserve(poly.xyz.size());
	for (std::size_t i = 0; i + 2 < poly.xyz.size(); i += 3)
	{
		out.polylineXy.push_back(poly.xyz[i]);
		out.polylineXy.push_back(poly.xyz[i + 1]);
		out.polylineXy.push_back(0.f);
	}
}

bool discretizeToEntity(const TopoDS_Edge& edge, const TessellateParams& params, DrawingEntity& out)
{
	Polyline3d poly;
	if (!discretizeEdge(edge, params, poly, nullptr) || poly.xyz.size() < 6)
		return false;
	fillPolylineXyFromPoly(poly, out);
	return out.polylineXy.size() >= 6;
}

} // namespace

bool drawingEntityFromEdge(const TopoDS_Edge& edge, DrawingEdgeClass cls, bool hidden,
						   const TessellateParams& params, DrawingEntity& out)
{
	out = DrawingEntity{};
	out.edgeClass = cls;
	out.hidden = hidden;
	out.kind = DrawingEntityKind::Polyline;
	if (edge.IsNull())
		return false;

	// HLR 边在投影系；圆平面未必∥投影面，按圆心+cos/sin 重采样会出飘弧/假圆
	if (!discretizeToEntity(edge, params, out))
		return false;

	try
	{
		const BRepAdaptor_Curve adapt(edge);
		const GeomAbs_CurveType ty = adapt.GetType();
		switch (ty)
		{
		case GeomAbs_Line:
		{
			out.kind = DrawingEntityKind::Line;
			out.data[0] = out.polylineXy[0];
			out.data[1] = out.polylineXy[1];
			out.data[3] = out.polylineXy[out.polylineXy.size() - 3];
			out.data[4] = out.polylineXy[out.polylineXy.size() - 2];
			break;
		}
		case GeomAbs_Circle:
		{
			const gp_Circ circ = adapt.Circle();
			// 仅面法向接近投影 Z 时标记圆元，供后续 DXF；几何仍用离散点
			if (std::fabs(circ.Axis().Direction().Z()) > 0.95)
			{
				const double f = adapt.FirstParameter();
				const double l = adapt.LastParameter();
				const gp_Pnt s = adapt.Value(f);
				const gp_Pnt e = adapt.Value(l);
				out.data[0] = circ.Location().X();
				out.data[1] = circ.Location().Y();
				out.data[3] = circ.Radius();
				if (std::fabs(l - f) > 1.0 && s.SquareDistance(e) < 0.001)
					out.kind = DrawingEntityKind::Circle;
				else
				{
					out.kind = DrawingEntityKind::ArcOfCircle;
					out.data[4] = f;
					out.data[5] = l;
				}
			}
			break;
		}
		case GeomAbs_Ellipse:
		{
			const gp_Elips el = adapt.Ellipse();
			if (std::fabs(el.Axis().Direction().Z()) > 0.95)
			{
				const double f = adapt.FirstParameter();
				const double l = adapt.LastParameter();
				const gp_Pnt s = adapt.Value(f);
				const gp_Pnt e = adapt.Value(l);
				out.data[0] = el.Location().X();
				out.data[1] = el.Location().Y();
				out.data[3] = el.MajorRadius();
				out.data[4] = el.MinorRadius();
				if (std::fabs(l - f) > 1.0 && s.SquareDistance(e) < 0.001)
					out.kind = DrawingEntityKind::Ellipse;
				else
				{
					out.kind = DrawingEntityKind::ArcOfEllipse;
					out.data[6] = f;
					out.data[7] = l;
				}
			}
			break;
		}
		case GeomAbs_BSplineCurve:
			out.kind = DrawingEntityKind::BSpline;
			break;
		default:
			break;
		}
	}
	catch (...)
	{
		// 分类失败不影响已离散几何
	}
	return true;
}

} // namespace geoalgo
