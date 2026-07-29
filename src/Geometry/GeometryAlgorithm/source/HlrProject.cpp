/// @file HlrProject.cpp
/// @brief OCC HLR / 剖切投影为图面折线

#include "HlrProject.h"

#include "Discretize.h"
#include "ShapeHandle.h"

#include <BRep_Builder.hxx>
#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <HLRAlgo_Projector.hxx>
#include <HLRBRep_Algo.hxx>
#include <HLRBRep_HLRToShape.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>

#include <cmath>

namespace geoalgo
{
namespace
{

bool shapeCenterAndBox(const TopoDS_Shape& shape, gp_Pnt& outCenter, Bnd_Box& outBox, std::string* errMsg)
{
	BRepBndLib::Add(shape, outBox);
	if (outBox.IsVoid())
	{
		if (errMsg)
			*errMsg = "HLR: empty bounding box";
		return false;
	}
	Standard_Real xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
	outBox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
	outCenter = gp_Pnt(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax));
	return true;
}

gp_Ax2 projectorAx2(HlrViewKind kind, HlrProjectionAngle angle, const gp_Pnt& center)
{
	const bool third = (angle == HlrProjectionAngle::Third);
	switch (kind)
	{
	case HlrViewKind::Top:
		// 第一角：自上向下；第三角：自下向上
		return third ? gp_Ax2(center, gp_Dir(0.0, 0.0, 1.0), gp_Dir(1.0, 0.0, 0.0))
					 : gp_Ax2(center, gp_Dir(0.0, 0.0, -1.0), gp_Dir(1.0, 0.0, 0.0));
	case HlrViewKind::Right:
		return third ? gp_Ax2(center, gp_Dir(-1.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0))
					 : gp_Ax2(center, gp_Dir(1.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0));
	case HlrViewKind::Iso:
	{
		const double invSqrt3 = 1.0 / std::sqrt(3.0);
		const gp_Dir view(invSqrt3, invSqrt3, invSqrt3);
		gp_Dir xDir(-1.0, 1.0, 0.0);
		return gp_Ax2(center, view, xDir);
	}
	case HlrViewKind::Front:
	default:
		return third ? gp_Ax2(center, gp_Dir(0.0, 1.0, 0.0), gp_Dir(1.0, 0.0, 0.0))
					 : gp_Ax2(center, gp_Dir(0.0, -1.0, 0.0), gp_Dir(1.0, 0.0, 0.0));
	}
}

void addShapeEdges(TopoDS_Compound& compound, BRep_Builder& builder, const TopoDS_Shape& shape)
{
	if (shape.IsNull())
		return;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		if (!edge.IsNull())
			builder.Add(compound, edge);
	}
}

void polylinesToDrawingXy(std::vector<Polyline3d>& polys)
{
	for (Polyline3d& poly : polys)
	{
		std::vector<float> xy;
		xy.reserve(poly.xyz.size());
		for (std::size_t i = 0; i + 2 < poly.xyz.size(); i += 3)
		{
			xy.push_back(poly.xyz[i]);
			xy.push_back(poly.xyz[i + 1]);
			xy.push_back(0.f);
		}
		poly.xyz.swap(xy);
	}
}

bool compoundToPolylines(const TopoDS_Shape& compound, const TessellateParams& params,
						 std::vector<Polyline3d>& out, std::string* errMsg)
{
	out.clear();
	if (compound.IsNull())
		return true;
	// HLR 复合体常含退化边，单边失败则跳过，避免整视图空白
	for (TopExp_Explorer exp(compound, TopAbs_EDGE); exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		Polyline3d poly;
		std::string localErr;
		if (!discretizeEdge(edge, params, poly, &localErr))
			continue;
		if (poly.xyz.size() >= 6)
			out.push_back(std::move(poly));
	}
	(void)errMsg;
	polylinesToDrawingXy(out);
	return true;
}

/// 丢掉非有限或远离模型尺度的投影点（右视图偶发 1e6 级野点会撑爆图幅）
void sanitizeDrawingPolylines(std::vector<Polyline3d>& polys, double maxAbsCoord)
{
	if (maxAbsCoord <= 0.0 || polys.empty())
		return;
	std::vector<Polyline3d> kept;
	kept.reserve(polys.size());
	for (Polyline3d& poly : polys)
	{
		bool ok = poly.xyz.size() >= 6;
		for (std::size_t i = 0; ok && i + 2 < poly.xyz.size(); i += 3)
		{
			const double x = poly.xyz[i];
			const double y = poly.xyz[i + 1];
			if (!std::isfinite(x) || !std::isfinite(y) || std::fabs(x) > maxAbsCoord || std::fabs(y) > maxAbsCoord)
				ok = false;
		}
		if (ok)
			kept.push_back(std::move(poly));
	}
	polys.swap(kept);
}

double shapeDiagonal(const Bnd_Box& box)
{
	Standard_Real xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
	box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
	const double dx = xmax - xmin;
	const double dy = ymax - ymin;
	const double dz = zmax - zmin;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

gp_Pln midPlane(const Bnd_Box& box, DrawingSectionPlane plane)
{
	Standard_Real xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
	box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
	const gp_Pnt c(0.5 * (xmin + xmax), 0.5 * (ymin + ymax), 0.5 * (zmin + zmax));
	switch (plane)
	{
	case DrawingSectionPlane::TopParallel:
		return gp_Pln(c, gp_Dir(0.0, 0.0, 1.0));
	case DrawingSectionPlane::RightParallel:
		return gp_Pln(c, gp_Dir(1.0, 0.0, 0.0));
	case DrawingSectionPlane::FrontParallel:
	default:
		return gp_Pln(c, gp_Dir(0.0, 1.0, 0.0));
	}
}

bool sectionShapeToDrawingPln(const ShapeHandle& shape, const gp_Pln& pln, const Bnd_Box& box,
							  const TessellateParams& params, HlrViewPolylines& out, std::string* errMsg)
{
	out = HlrViewPolylines{};
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		if (errMsg)
			*errMsg = "section: null ShapeHandle";
		return false;
	}

	Standard_Real xmin = 0, ymin = 0, zmin = 0, xmax = 0, ymax = 0, zmax = 0;
	box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
	const double diag =
		std::sqrt((xmax - xmin) * (xmax - xmin) + (ymax - ymin) * (ymax - ymin) + (zmax - zmin) * (zmax - zmin));
	const double planeSize = (std::max)(diag * 2.0, 100.0);

	const TopoDS_Face planeFace =
		BRepBuilderAPI_MakeFace(pln, -planeSize, planeSize, -planeSize, planeSize).Face();
	BRepAlgoAPI_Section sec(planeFace, native, Standard_False);
	sec.Approximation(Standard_True);
	sec.Build();
	if (!sec.IsDone())
	{
		if (errMsg)
			*errMsg = "section: BRepAlgoAPI_Section failed";
		return false;
	}

	const gp_Ax3 ax = pln.Position();
	std::vector<Polyline3d> edges3d;
	for (TopExp_Explorer exp(sec.Shape(), TopAbs_EDGE); exp.More(); exp.Next())
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		Polyline3d poly;
		if (!discretizeEdge(edge, params, poly, nullptr))
			continue;
		if (poly.xyz.size() >= 6)
			edges3d.push_back(std::move(poly));
	}
	if (edges3d.empty())
	{
		if (errMsg)
			*errMsg = "section: empty";
		return false;
	}
	out.visible.clear();
	out.visible.reserve(edges3d.size());
	for (const Polyline3d& e : edges3d)
	{
		Polyline3d xy;
		xy.xyz.reserve(e.xyz.size());
		for (std::size_t i = 0; i + 2 < e.xyz.size(); i += 3)
		{
			const gp_Pnt p(e.xyz[i], e.xyz[i + 1], e.xyz[i + 2]);
			const gp_Vec v(ax.Location(), p);
			xy.xyz.push_back(static_cast<float>(v.Dot(gp_Vec(ax.XDirection()))));
			xy.xyz.push_back(static_cast<float>(v.Dot(gp_Vec(ax.YDirection()))));
			xy.xyz.push_back(0.f);
		}
		if (xy.xyz.size() >= 6)
			out.visible.push_back(std::move(xy));
	}
	if (out.visible.empty())
	{
		if (errMsg)
			*errMsg = "section: no edges";
		return false;
	}
	return true;
}

} // namespace

bool projectShapeHlr(const ShapeHandle& shape, HlrViewKind kind, HlrProjectionAngle angle,
					 const TessellateParams& params, HlrViewPolylines& out, std::string* errMsg)
{
	out = HlrViewPolylines{};
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		if (errMsg)
			*errMsg = "HLR: null ShapeHandle";
		return false;
	}

	gp_Pnt center;
	Bnd_Box box;
	if (!shapeCenterAndBox(native, center, box, errMsg))
		return false;

	Handle(HLRBRep_Algo) hlr = new HLRBRep_Algo();
	hlr->Add(native);
	hlr->Projector(HLRAlgo_Projector(projectorAx2(kind, angle, center)));
	hlr->Update();
	hlr->Hide();

	HLRBRep_HLRToShape toShape(hlr);
	BRep_Builder builder;
	TopoDS_Compound visibleCompound;
	TopoDS_Compound hiddenCompound;
	builder.MakeCompound(visibleCompound);
	builder.MakeCompound(hiddenCompound);

	addShapeEdges(visibleCompound, builder, toShape.VCompound());
	addShapeEdges(visibleCompound, builder, toShape.OutLineVCompound());
	addShapeEdges(visibleCompound, builder, toShape.Rg1LineVCompound());
	addShapeEdges(hiddenCompound, builder, toShape.HCompound());
	addShapeEdges(hiddenCompound, builder, toShape.OutLineHCompound());

	if (!compoundToPolylines(visibleCompound, params, out.visible, errMsg))
		return false;
	if (!compoundToPolylines(hiddenCompound, params, out.hidden, errMsg))
		return false;

	const double limit = (std::max)(shapeDiagonal(box) * 50.0, 1000.0);
	sanitizeDrawingPolylines(out.visible, limit);
	sanitizeDrawingPolylines(out.hidden, limit);

	if (out.visible.empty() && out.hidden.empty())
	{
		if (errMsg)
			*errMsg = "HLR: no projection edges";
		return false;
	}
	return true;
}

bool projectShapeHlr(const ShapeHandle& shape, HlrViewKind kind, const TessellateParams& params, HlrViewPolylines& out,
					 std::string* errMsg)
{
	return projectShapeHlr(shape, kind, HlrProjectionAngle::First, params, out, errMsg);
}

bool projectShapeHlrThreeViews(const ShapeHandle& shape, HlrProjectionAngle angle, const TessellateParams& params,
							   HlrThreeViewsResult& out, std::string* errMsg)
{
	out = HlrThreeViewsResult{};
	if (!projectShapeHlr(shape, HlrViewKind::Front, angle, params, out.front, errMsg))
		return false;
	if (!projectShapeHlr(shape, HlrViewKind::Top, angle, params, out.top, errMsg))
		return false;
	if (!projectShapeHlr(shape, HlrViewKind::Right, angle, params, out.right, errMsg))
		return false;
	return true;
}

bool projectShapeHlrThreeViews(const ShapeHandle& shape, const TessellateParams& params, HlrThreeViewsResult& out,
							   std::string* errMsg)
{
	return projectShapeHlrThreeViews(shape, HlrProjectionAngle::First, params, out, errMsg);
}

bool sectionShapeToDrawing(const ShapeHandle& shape, DrawingSectionPlane plane, const TessellateParams& params,
						   HlrViewPolylines& out, std::string* errMsg)
{
	out = HlrViewPolylines{};
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		if (errMsg)
			*errMsg = "section: null ShapeHandle";
		return false;
	}
	gp_Pnt center;
	Bnd_Box box;
	if (!shapeCenterAndBox(native, center, box, errMsg))
		return false;
	return sectionShapeToDrawingPln(shape, midPlane(box, plane), box, params, out, errMsg);
}

bool sectionShapeToDrawing(const ShapeHandle& shape, const double originMm[3], const double normal[3],
						   const TessellateParams& params, HlrViewPolylines& out, std::string* errMsg)
{
	out = HlrViewPolylines{};
	if (!originMm || !normal)
	{
		if (errMsg)
			*errMsg = "section: null origin/normal";
		return false;
	}
	const double nl = std::sqrt(normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]);
	if (nl < 1e-12)
	{
		if (errMsg)
			*errMsg = "section: zero normal";
		return false;
	}
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		if (errMsg)
			*errMsg = "section: null ShapeHandle";
		return false;
	}
	gp_Pnt center;
	Bnd_Box box;
	if (!shapeCenterAndBox(native, center, box, errMsg))
		return false;
	const gp_Pln pln(gp_Pnt(originMm[0], originMm[1], originMm[2]),
					gp_Dir(normal[0] / nl, normal[1] / nl, normal[2] / nl));
	return sectionShapeToDrawingPln(shape, pln, box, params, out, errMsg);
}

bool projectShapeHlrDrawingBundle(const ShapeHandle& shape, HlrProjectionAngle angle, bool includeIso,
								  bool includeSection, DrawingSectionPlane sectionPlane,
								  const TessellateParams& params, HlrDrawingBundle& out, std::string* errMsg)
{
	return projectShapeHlrDrawingBundle(shape, angle, includeIso, includeSection, sectionPlane, false, nullptr,
										nullptr, params, out, errMsg);
}

bool projectShapeHlrDrawingBundle(const ShapeHandle& shape, HlrProjectionAngle angle, bool includeIso,
								  bool includeSection, DrawingSectionPlane sectionPlane, bool customSection,
								  const double originMm[3], const double normal[3], const TessellateParams& params,
								  HlrDrawingBundle& out, std::string* errMsg)
{
	out = HlrDrawingBundle{};
	HlrThreeViewsResult three;
	if (!projectShapeHlrThreeViews(shape, angle, params, three, errMsg))
		return false;
	out.front = std::move(three.front);
	out.top = std::move(three.top);
	out.right = std::move(three.right);
	if (includeIso)
	{
		if (projectShapeHlr(shape, HlrViewKind::Iso, angle, params, out.iso, nullptr))
			out.hasIso = true;
	}
	if (includeSection)
	{
		bool ok = false;
		if (customSection && originMm && normal)
			ok = sectionShapeToDrawing(shape, originMm, normal, params, out.section, nullptr);
		else
			ok = sectionShapeToDrawing(shape, sectionPlane, params, out.section, nullptr);
		if (ok)
			out.hasSection = true;
	}
	return true;
}

} // namespace geoalgo
