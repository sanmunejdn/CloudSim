/// @file SketchSweep.cpp
/// @brief 闭合轮廓沿路径扫描（显式对齐截面 + 单边 spine + PipeShell/MakePipe）

#include "SketchSweep.h"

#include "BrepBoolean.h"
#include "detail/OccIncludes.h"
#include "detail/SketchCurveWireOcc.h"

#include <BRepAdaptor_CompCurve.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepGProp.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepTools.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GProp_GProps.hxx>
#include <GProp_PEquation.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomFill_Trihedron.hxx>
#include <Geom_BSplineCurve.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Ax1.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <functional>
#include <vector>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace geoalgo
{
namespace
{
constexpr double kEps = 1e-9;
constexpr double kAlignDotMin = 0.98; // 对齐后法向与路径切向夹角余弦下限

void appendUnique(std::vector<gp_Pnt>& pts, const gp_Pnt& p)
{
	if (pts.empty() || pts.back().Distance(p) > kEps)
		pts.push_back(p);
}

bool makeClosedFace(const std::vector<float>& xyz, TopoDS_Face& outFace, std::string* errMsg)
{
	double nx = 0, ny = 0, nz = 1;
	(void)estimatePolylinePlaneNormal(xyz, nx, ny, nz);
	return makeClosedFaceFromPolylineMm(xyz, nx, ny, nz, outFace, errMsg);
}

bool makeProfileFace(const std::vector<float>& xyz, const SketchSweepParams& params, TopoDS_Face& outFace,
					 std::string* errMsg)
{
	if (!params.profileSegments.empty())
	{
		double nx = 0, ny = 0, nz = 1;
		if (xyz.size() >= 9)
			(void)estimatePolylinePlaneNormal(xyz, nx, ny, nz);
		else if (params.profileSegments[0].kind == SketchCurveSegKind::Circle
				 || params.profileSegments[0].kind == SketchCurveSegKind::Ellipse)
		{
			const auto& s = params.profileSegments[0];
			const double len = std::sqrt(s.mx * s.mx + s.my * s.my + s.mz * s.mz);
			if (len > 1e-12)
			{
				nx = s.mx / len;
				ny = s.my / len;
				nz = s.mz / len;
			}
		}
		return makeClosedFaceFromSegments(params.profileSegments, nx, ny, nz, outFace, errMsg);
	}
	return makeClosedFace(xyz, outFace, errMsg);
}

bool fitBSplineEdge(const std::vector<gp_Pnt>& pts, TopoDS_Edge& outEdge)
{
	if (pts.size() < 2U)
		return false;
	if (pts.size() == 2U)
	{
		BRepBuilderAPI_MakeEdge mk(pts[0], pts[1]);
		if (!mk.IsDone())
			return false;
		outEdge = mk.Edge();
		return !outEdge.IsNull();
	}
	TColgp_Array1OfPnt arr(1, static_cast<int>(pts.size()));
	for (int i = 1; i <= arr.Upper(); ++i)
		arr.SetValue(i, pts[static_cast<std::size_t>(i - 1)]);
	for (int degMin : {3, 2, 1})
	{
		GeomAPI_PointsToBSpline fitter(arr, degMin, 8, GeomAbs_C2, 1.0e-3);
		if (!fitter.IsDone())
			continue;
		const Handle(Geom_BSplineCurve) curve = fitter.Curve();
		if (curve.IsNull())
			continue;
		BRepBuilderAPI_MakeEdge edgeMaker(curve);
		if (!edgeMaker.IsDone())
			continue;
		outEdge = edgeMaker.Edge();
		return !outEdge.IsNull();
	}
	return false;
}

bool makeSingleEdgeSpine(const std::vector<gp_Pnt>& pts, TopoDS_Wire& outWire, std::string* errMsg)
{
	TopoDS_Edge edge;
	if (!fitBSplineEdge(pts, edge))
	{
		if (errMsg)
			*errMsg = "path spine fit failed";
		return false;
	}
	BRepBuilderAPI_MakeWire wireMaker(edge);
	if (!wireMaker.IsDone())
	{
		if (errMsg)
			*errMsg = "path spine wire failed";
		return false;
	}
	outWire = wireMaker.Wire();
	return !outWire.IsNull();
}

bool makePathWireFromPolyline(const std::vector<float>& xyz, TopoDS_Wire& outWire, std::string* errMsg)
{
	if (xyz.size() < 6U || (xyz.size() % 3U) != 0U)
	{
		if (errMsg)
			*errMsg = "path needs >=2 points";
		return false;
	}
	std::vector<gp_Pnt> pts;
	const std::size_t n = xyz.size() / 3U;
	pts.reserve(n);
	for (std::size_t i = 0; i < n; ++i)
		appendUnique(pts, gp_Pnt(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
	return makeSingleEdgeSpine(pts, outWire, errMsg);
}

bool makePathWireFromSegments(const std::vector<SketchSweepPathSegment>& segs, TopoDS_Wire& outWire,
							  std::string* errMsg)
{
	if (segs.empty())
	{
		if (errMsg)
			*errMsg = "path segments empty";
		return false;
	}

	bool hasArc = false;
	for (const auto& s : segs)
	{
		if (s.kind == SketchSweepPathSegKind::Arc)
		{
			hasArc = true;
			break;
		}
	}

	// 无圆弧：一律拟合成单边 BSpline，避免多段折线 spine 切成平面扇片
	if (!hasArc)
	{
		std::vector<gp_Pnt> pts;
		pts.reserve(segs.size() + 1U);
		for (const auto& s : segs)
		{
			appendUnique(pts, gp_Pnt(s.ax, s.ay, s.az));
			appendUnique(pts, gp_Pnt(s.bx, s.by, s.bz));
		}
		return makeSingleEdgeSpine(pts, outWire, errMsg);
	}

	BRepBuilderAPI_MakeWire wireMaker;
	int edges = 0;
	std::vector<gp_Pnt> run;
	auto flushRun = [&]() -> bool
	{
		if (run.size() < 2U)
		{
			run.clear();
			return true;
		}
		TopoDS_Edge e;
		if (!fitBSplineEdge(run, e))
			return false;
		wireMaker.Add(e);
		if (!wireMaker.IsDone())
			return false;
		++edges;
		run.clear();
		return true;
	};

	for (const auto& s : segs)
	{
		const gp_Pnt a(s.ax, s.ay, s.az);
		const gp_Pnt b(s.bx, s.by, s.bz);
		if (s.kind == SketchSweepPathSegKind::Arc)
		{
			if (!flushRun())
			{
				if (errMsg)
					*errMsg = "path smooth run failed";
				return false;
			}
			GC_MakeArcOfCircle mkArc(a, gp_Pnt(s.mx, s.my, s.mz), b);
			if (!mkArc.IsDone())
			{
				if (errMsg)
					*errMsg = "path GC_MakeArcOfCircle failed";
				return false;
			}
			BRepBuilderAPI_MakeEdge edgeMaker(mkArc.Value());
			if (!edgeMaker.IsDone())
			{
				if (errMsg)
					*errMsg = "path arc MakeEdge failed";
				return false;
			}
			wireMaker.Add(edgeMaker.Edge());
			if (!wireMaker.IsDone())
			{
				if (errMsg)
					*errMsg = "path edges do not form a wire";
				return false;
			}
			++edges;
			continue;
		}
		appendUnique(run, a);
		appendUnique(run, b);
	}
	if (!flushRun())
	{
		if (errMsg)
			*errMsg = "path smooth run failed";
		return false;
	}
	if (edges == 0)
	{
		if (errMsg)
			*errMsg = "path has no usable edges";
		return false;
	}
	outWire = wireMaker.Wire();
	return !outWire.IsNull();
}

gp_Pnt profileCentroid(const TopoDS_Face& face)
{
	GProp_GProps props;
	BRepGProp::SurfaceProperties(face, props);
	if (props.Mass() > kEps)
		return props.CentreOfMass();
	BRepAdaptor_Surface surf(face, Standard_True);
	return surf.Plane().Location();
}

bool spineEnds(const TopoDS_Wire& pathWire, gp_Pnt& outP0, gp_Pnt& outP1, gp_Dir& outT0)
{
	try
	{
		BRepAdaptor_CompCurve cc(pathWire);
		const double u0 = cc.FirstParameter();
		const double u1 = cc.LastParameter();
		outP0 = cc.Value(u0);
		outP1 = cc.Value(u1);
		// 略向内取切向，避开端点数值抖
		const double uT = u0 + 1e-4 * (u1 - u0);
		gp_Pnt p;
		gp_Vec v;
		cc.D1(uT, p, v);
		if (v.Magnitude() < 1e-12)
			cc.D1(u0, p, v);
		if (v.Magnitude() < 1e-12)
			return false;
		outT0 = gp_Dir(v);
		return true;
	}
	catch (const Standard_Failure&)
	{
		return false;
	}
}

void orientSpineNearProfile(const TopoDS_Face& profileFace, TopoDS_Wire& pathWire)
{
	const gp_Pnt center = profileCentroid(profileFace);
	gp_Pnt p0, p1;
	gp_Dir t0;
	if (!spineEnds(pathWire, p0, p1, t0))
		return;
	if (p1.SquareDistance(center) + 1e-12 < p0.SquareDistance(center))
		pathWire.Reverse();
}

bool tryPlanarPathBinormal(const TopoDS_Wire& pathWire, gp_Dir& outBi)
{
	std::vector<gp_Pnt> pts;
	for (TopExp_Explorer ex(pathWire, TopAbs_EDGE); ex.More(); ex.Next())
	{
		BRepAdaptor_Curve c(TopoDS::Edge(ex.Current()));
		constexpr int n = 16;
		for (int i = 0; i <= n; ++i)
		{
			const double u =
				c.FirstParameter() + (c.LastParameter() - c.FirstParameter()) * static_cast<double>(i) / n;
			appendUnique(pts, c.Value(u));
		}
	}
	if (pts.size() < 3U)
		return false;
	double span = 0.0;
	for (std::size_t i = 1; i < pts.size(); ++i)
		span = (std::max)(span, pts.front().Distance(pts[i]));
	const double tol = (std::max)(1.0e-4, span * 1.0e-5);
	TColgp_Array1OfPnt arr(1, static_cast<int>(pts.size()));
	for (int i = 1; i <= arr.Upper(); ++i)
		arr.SetValue(i, pts[static_cast<std::size_t>(i - 1)]);
	GProp_PEquation eq(arr, tol);
	if (!eq.IsPlanar())
		return false;
	outBi = eq.Plane().Axis().Direction();
	return true;
}

bool profilePlaneNormal(const TopoDS_Face& face, gp_Dir& outN)
{
	BRepAdaptor_Surface surf(face, Standard_True);
	if (surf.GetType() != GeomAbs_Plane)
		return false;
	outN = surf.Plane().Position().Direction();
	return true;
}

// 平移中心到路径起点，再绕 n×t 旋转使法向对齐切向（不用 SetTransformation，避免坐标系语义搞反）
bool placeProfileAtSpineStart(const TopoDS_Face& profileIn, const TopoDS_Wire& pathWire, TopoDS_Face& outFace,
							  std::string* errMsg)
{
	gp_Pnt p0, p1;
	gp_Dir t0;
	if (!spineEnds(pathWire, p0, p1, t0))
	{
		if (errMsg)
			*errMsg = "path start frame failed";
		return false;
	}

	gp_Dir n;
	if (!profilePlaneNormal(profileIn, n))
	{
		outFace = profileIn;
		return true;
	}
	if (n.Dot(t0) < 0.0)
		n.Reverse();

	const gp_Pnt center = profileCentroid(profileIn);

	gp_Trsf move;
	move.SetTranslation(gp_Vec(center, p0));

	gp_Trsf rot;
	const double align = std::abs(n.Dot(t0));
	if (align < kAlignDotMin)
	{
		gp_Vec axis = gp_Vec(n).Crossed(gp_Vec(t0));
		double angle = n.Angle(t0);
		if (axis.Magnitude() < 1e-12)
		{
			// 反向平行：绕任意垂直轴转 180°
			gp_Vec arb(1.0, 0.0, 0.0);
			if (std::abs(n.Dot(gp_Dir(arb))) > 0.9)
				arb = gp_Vec(0.0, 1.0, 0.0);
			axis = gp_Vec(n).Crossed(arb);
			angle = M_PI;
		}
		if (axis.Magnitude() < 1e-12)
		{
			if (errMsg)
				*errMsg = "profile rotate axis degenerate";
			return false;
		}
		rot.SetRotation(gp_Ax1(p0, gp_Dir(axis)), angle);
	}

	const gp_Trsf tr = rot * move;
	BRepBuilderAPI_Transform xf(profileIn, tr, Standard_True);
	if (!xf.IsDone())
	{
		if (errMsg)
			*errMsg = "profile align transform failed";
		return false;
	}
	outFace = TopoDS::Face(xf.Shape());
	if (outFace.IsNull())
	{
		if (errMsg)
			*errMsg = "profile align produced null face";
		return false;
	}

	gp_Dir n2;
	if (profilePlaneNormal(outFace, n2) && std::abs(n2.Dot(t0)) < kAlignDotMin)
	{
		if (errMsg)
			*errMsg = "profile not perpendicular to path after align";
		return false;
	}
	return true;
}

// MVP：对齐后绕路径起点切向预旋转截面
bool applyTwistAtSpineStart(const TopoDS_Face& profileIn, const TopoDS_Wire& pathWire, double twistDeg,
							TopoDS_Face& outFace, std::string* errMsg)
{
	if (std::abs(twistDeg) <= 1e-6)
	{
		outFace = profileIn;
		return true;
	}
	gp_Pnt p0, p1;
	gp_Dir t0;
	if (!spineEnds(pathWire, p0, p1, t0))
	{
		if (errMsg)
			*errMsg = "twist: path start frame failed";
		return false;
	}
	gp_Trsf rot;
	rot.SetRotation(gp_Ax1(p0, t0), twistDeg * M_PI / 180.0);
	BRepBuilderAPI_Transform xf(profileIn, rot, Standard_True);
	if (!xf.IsDone())
	{
		if (errMsg)
			*errMsg = "twist transform failed";
		return false;
	}
	outFace = TopoDS::Face(xf.Shape());
	return !outFace.IsNull();
}

bool spineLooksCurved(const TopoDS_Wire& pathWire)
{
	for (TopExp_Explorer ex(pathWire, TopAbs_EDGE); ex.More(); ex.Next())
	{
		BRepAdaptor_Curve c(TopoDS::Edge(ex.Current()));
		if (c.GetType() != GeomAbs_Line)
			return true;
	}
	return false;
}

bool shapeLooksLikeFacetedSweep(const TopoDS_Shape& shape, bool curvedSpine)
{
	if (shape.IsNull())
		return true;
	int faces = 0;
	int planar = 0;
	for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next())
	{
		++faces;
		BRepAdaptor_Surface surf(TopoDS::Face(ex.Current()), Standard_True);
		if (surf.GetType() == GeomAbs_Plane)
			++planar;
	}
	if (faces <= 0)
		return true;
	if (curvedSpine && planar == faces && faces > 4)
		return true;
	if (curvedSpine && faces >= 6 && planar >= faces - 1)
		return true;
	return false;
}

void unifyCoplanarFaces(TopoDS_Shape& shape)
{
	if (shape.IsNull())
		return;
	ShapeUpgrade_UnifySameDomain unify(shape, Standard_True, Standard_True, Standard_True);
	unify.Build();
	const TopoDS_Shape u = unify.Shape();
	if (!u.IsNull())
		shape = u;
}

bool buildPipeSolid(const TopoDS_Face& profileFace, const TopoDS_Wire& pathWire, TopoDS_Shape& outTool,
					std::string* errMsg)
{
	try
	{
		const TopoDS_Wire profileWire = BRepTools::OuterWire(profileFace);
		if (profileWire.IsNull())
		{
			if (errMsg)
				*errMsg = "profile has no outer wire";
			return false;
		}

		const bool curved = spineLooksCurved(pathWire);
		gp_Dir biNormal;
		const bool planarPath = tryPlanarPathBinormal(pathWire, biNormal);

		auto accept = [&](const TopoDS_Shape& s) -> bool
		{
			if (s.IsNull() || shapeLooksLikeFacetedSweep(s, curved))
				return false;
			outTool = s;
			return true;
		};

		auto tryShell = [&](const std::function<void(BRepOffsetAPI_MakePipeShell&)>& setMode) -> bool
		{
			BRepOffsetAPI_MakePipeShell shell(pathWire);
			shell.SetTolerance(1.0e-4, 1.0e-4, 1.0e-2);
			setMode(shell);
			// 截面已放到起点：不再 WithContact，避免被再次拖偏
			shell.Add(profileWire, Standard_False, Standard_True);
			if (!shell.IsReady())
				return false;
			shell.Build();
			if (!shell.IsDone())
				return false;
			(void)shell.MakeSolid();
			return accept(shell.Shape());
		};

		if (planarPath)
		{
			if (tryShell([&](BRepOffsetAPI_MakePipeShell& s) { s.SetMode(biNormal); }))
				return true;
		}
		// CorrectedFrenet（IsFrenet=false）
		if (tryShell([](BRepOffsetAPI_MakePipeShell& s) { s.SetMode(Standard_False); }))
			return true;
		if (tryShell([](BRepOffsetAPI_MakePipeShell& s) { s.SetDiscreteMode(); }))
			return true;

		{
			BRepOffsetAPI_MakePipe pipe(pathWire, profileFace, GeomFill_IsCorrectedFrenet, Standard_True);
			if (pipe.IsDone() && accept(pipe.Shape()))
				return true;
		}
		{
			BRepOffsetAPI_MakePipe pipe(pathWire, profileWire, GeomFill_IsCorrectedFrenet, Standard_True);
			if (pipe.IsDone() && accept(pipe.Shape()))
				return true;
		}

		if (errMsg)
			*errMsg = "MakePipe/MakePipeShell failed";
		return false;
	}
	catch (const Standard_Failure& e)
	{
		if (errMsg)
		{
			const char* m = e.GetMessageString();
			*errMsg = m ? (std::string("OCC sweep: ") + m) : "OCC sweep failure";
		}
		return false;
	}
	catch (...)
	{
		if (errMsg)
			*errMsg = "OCC sweep: unknown exception";
		return false;
	}
}

bool pipeAndBoolean(const TopoDS_Face& profileFaceIn, const TopoDS_Wire& pathWireIn, const SketchSweepParams& params,
					const ShapeHandle* baseOrNull, ShapeHandle& outShape, std::string* errMsg)
{
	TopoDS_Wire pathWire = pathWireIn;
	orientSpineNearProfile(profileFaceIn, pathWire);

	TopoDS_Face profileFace;
	if (!placeProfileAtSpineStart(profileFaceIn, pathWire, profileFace, errMsg))
		return false;

	TopoDS_Face twistedFace;
	if (!applyTwistAtSpineStart(profileFace, pathWire, params.twistDeg, twistedFace, errMsg))
		return false;

	TopoDS_Shape tool;
	if (!buildPipeSolid(twistedFace, pathWire, tool, errMsg))
		return false;
	unifyCoplanarFaces(tool);

	TopoDS_Shape baseNative;
	const TopoDS_Shape* basePtr = nullptr;
	if (baseOrNull && !baseOrNull->isNull())
	{
		if (!ShapeHandleAccess::nativeShape(*baseOrNull, &baseNative) || baseNative.IsNull())
		{
			if (errMsg)
				*errMsg = "invalid base ShapeHandle";
			return false;
		}
		basePtr = &baseNative;
	}

	TopoDS_Shape result;
	if (params.mode == SketchSweepMode::Boss)
	{
		if (!basePtr)
			result = tool;
		else if (!brepBooleanToShape(*basePtr, tool, BrepBooleanOp::Fuse, result, errMsg))
			return false;
	}
	else
	{
		if (!basePtr)
		{
			if (errMsg)
				*errMsg = "SweepCut requires base solid";
			return false;
		}
		if (!brepBooleanToShape(*basePtr, tool, BrepBooleanOp::Cut, result, errMsg))
			return false;
	}

	unifyCoplanarFaces(result);
	outShape = ShapeHandleAccess::fromNativeShape(&result);
	return !outShape.isNull();
}
} // namespace

bool sketchSweepPolylineToHandle(const std::vector<float>& profilePolylineXyzMm,
								 const std::vector<float>& pathPolylineXyzMm, const SketchSweepParams& params,
								 const ShapeHandle* baseOrNull, ShapeHandle& outShape, std::string* errMsg)
{
	TopoDS_Face profileFace;
	if (!makeProfileFace(profilePolylineXyzMm, params, profileFace, errMsg))
		return false;
	TopoDS_Wire pathWire;
	if (!makePathWireFromPolyline(pathPolylineXyzMm, pathWire, errMsg))
		return false;
	return pipeAndBoolean(profileFace, pathWire, params, baseOrNull, outShape, errMsg);
}

bool sketchSweepSegmentsToHandle(const std::vector<float>& profilePolylineXyzMm,
								 const std::vector<SketchSweepPathSegment>& pathSegments,
								 const SketchSweepParams& params, const ShapeHandle* baseOrNull, ShapeHandle& outShape,
								 std::string* errMsg)
{
	TopoDS_Face profileFace;
	if (!makeProfileFace(profilePolylineXyzMm, params, profileFace, errMsg))
		return false;
	TopoDS_Wire pathWire;
	if (!makePathWireFromSegments(pathSegments, pathWire, errMsg))
		return false;
	return pipeAndBoolean(profileFace, pathWire, params, baseOrNull, outShape, errMsg);
}

} // namespace geoalgo
