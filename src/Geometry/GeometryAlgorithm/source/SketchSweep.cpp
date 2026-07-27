/// @file SketchSweep.cpp
/// @brief 轮廓沿路径 MakePipe（参考 OneCAD buildSweep）

#include "SketchSweep.h"

#include "BrepBoolean.h"
#include "detail/OccIncludes.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepOffsetAPI_MakePipe.hxx>
#include <BRepOffsetAPI_MakePipeShell.hxx>
#include <BRepTools.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <GProp_PEquation.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomFill_Trihedron.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Precision.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <Standard_Failure.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <algorithm>
#include <vector>

namespace geoalgo
{
namespace
{
constexpr double kPathEps = 1e-9;
constexpr double kSharpCornerCos = 0.642787609687; // cos(50°)：轮廓尖角切断
// 折线路径仅在接近直角时保留多边（L 形）；缓弯一律拟合成单条 BSpline，避免侧壁平面条带
constexpr double kOrthoCornerCos = 0.258819045103; // cos(75°)：|dot|<此值视为近直角

bool addBSplineEdge(BRepBuilderAPI_MakeWire& wireMaker, const std::vector<gp_Pnt>& run)
{
	if (run.size() < 2U)
		return false;
	if (run.size() == 2U)
	{
		BRepBuilderAPI_MakeEdge edgeMaker(run[0], run[1]);
		if (!edgeMaker.IsDone())
			return false;
		wireMaker.Add(edgeMaker.Edge());
		return wireMaker.IsDone();
	}
	TColgp_Array1OfPnt arr(1, static_cast<int>(run.size()));
	for (int i = 1; i <= arr.Upper(); ++i)
		arr.SetValue(i, run[static_cast<std::size_t>(i - 1)]);
	for (int degMin : {3, 2, 1})
	{
		// C1 足够光滑；C2 在密采样点上易抖出折皱
		GeomAPI_PointsToBSpline fitter(arr, degMin, 8, GeomAbs_C1, 1.0e-3);
		if (!fitter.IsDone())
			continue;
		const Handle(Geom_BSplineCurve) curve = fitter.Curve();
		if (curve.IsNull())
			continue;
		BRepBuilderAPI_MakeEdge edgeMaker(curve);
		if (!edgeMaker.IsDone())
			continue;
		wireMaker.Add(edgeMaker.Edge());
		return wireMaker.IsDone();
	}
	return false;
}

bool makeClosedFace(const std::vector<float>& xyz, TopoDS_Face& outFace, std::string* errMsg)
{
	if (xyz.size() < 9U || (xyz.size() % 3U) != 0U)
	{
		if (errMsg)
			*errMsg = "profile needs >=3 points";
		return false;
	}
	std::vector<gp_Pnt> pts;
	const std::size_t nIn = xyz.size() / 3U;
	pts.reserve(nIn);
	for (std::size_t i = 0; i < nIn; ++i)
	{
		const gp_Pnt p(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
		if (!pts.empty() && pts.back().Distance(p) < kPathEps)
			continue;
		pts.push_back(p);
	}
	if (pts.size() >= 2U && pts.front().Distance(pts.back()) < 1e-6)
		pts.pop_back();
	if (pts.size() < 3U)
	{
		if (errMsg)
			*errMsg = "profile needs >=3 points";
		return false;
	}

	auto turnCos = [](const gp_Pnt& a, const gp_Pnt& b, const gp_Pnt& c) -> double
	{
		gp_Vec v0(a, b);
		gp_Vec v1(b, c);
		if (v0.Magnitude() < kPathEps || v1.Magnitude() < kPathEps)
			return 1.0;
		v0.Normalize();
		v1.Normalize();
		return v0.Dot(v1);
	};

	// 在尖角处切断，平滑段拟合成单条 BSpline，避免侧壁被折线切碎
	std::vector<std::size_t> corners;
	for (std::size_t i = 0; i < pts.size(); ++i)
	{
		const gp_Pnt& a = pts[(i + pts.size() - 1) % pts.size()];
		const gp_Pnt& b = pts[i];
		const gp_Pnt& c = pts[(i + 1) % pts.size()];
		if (turnCos(a, b, c) < kSharpCornerCos)
			corners.push_back(i);
	}

	BRepBuilderAPI_MakeWire wireMaker;
	auto flushRun = [&](const std::vector<gp_Pnt>& run) -> bool
	{
		if (run.size() < 2U)
			return true;
		return addBSplineEdge(wireMaker, run);
	};

	// 矩形等全尖角轮廓：直接多边形面，避免边拟合干扰 MakePipeShell
	if (!corners.empty() && corners.size() == pts.size())
	{
		BRepBuilderAPI_MakePolygon poly;
		for (const auto& p : pts)
			poly.Add(p);
		poly.Add(pts.front());
		poly.Close();
		if (!poly.IsDone())
		{
			if (errMsg)
				*errMsg = "profile MakePolygon failed";
			return false;
		}
		BRepBuilderAPI_MakeFace mkFace(poly.Wire(), Standard_True);
		if (!mkFace.IsDone())
		{
			if (errMsg)
				*errMsg = "profile MakeFace failed";
			return false;
		}
		outFace = mkFace.Face();
		return true;
	}

	if (corners.empty())
	{
		// 全光滑闭环：过点 BSpline（首尾相接）成单边
		std::vector<gp_Pnt> loop = pts;
		loop.push_back(pts.front());
		BRepBuilderAPI_MakeWire w;
		if (addBSplineEdge(w, loop) && w.IsDone())
		{
			BRepBuilderAPI_MakeFace mkFace(w.Wire(), Standard_True);
			if (mkFace.IsDone())
			{
				outFace = mkFace.Face();
				return true;
			}
		}
		// 拟合失败则退回折线面
		BRepBuilderAPI_MakePolygon poly;
		for (const auto& p : pts)
			poly.Add(p);
		poly.Add(pts.front());
		poly.Close();
		if (!poly.IsDone())
		{
			if (errMsg)
				*errMsg = "profile MakePolygon failed";
			return false;
		}
		BRepBuilderAPI_MakeFace mkFace(poly.Wire(), Standard_True);
		if (!mkFace.IsDone())
		{
			if (errMsg)
				*errMsg = "profile MakeFace failed";
			return false;
		}
		outFace = mkFace.Face();
		return true;
	}

	for (std::size_t ci = 0; ci < corners.size(); ++ci)
	{
		const std::size_t i0 = corners[ci];
		const std::size_t i1 = corners[(ci + 1) % corners.size()];
		std::vector<gp_Pnt> run;
		run.push_back(pts[i0]);
		std::size_t j = (i0 + 1) % pts.size();
		while (true)
		{
			run.push_back(pts[j]);
			if (j == i1)
				break;
			j = (j + 1) % pts.size();
			if (run.size() > pts.size() + 2U)
				break;
		}
		if (!flushRun(run))
		{
			if (errMsg)
				*errMsg = "profile smooth-edge build failed";
			return false;
		}
	}
	if (!wireMaker.IsDone())
	{
		if (errMsg)
			*errMsg = "profile wire failed";
		return false;
	}
	BRepBuilderAPI_MakeFace mkFace(wireMaker.Wire(), Standard_True);
	if (!mkFace.IsDone())
	{
		if (errMsg)
			*errMsg = "profile MakeFace failed";
		return false;
	}
	outFace = mkFace.Face();
	return true;
}

bool fitPointsToBSplineEdge(const std::vector<gp_Pnt>& pts, TopoDS_Edge& outEdge)
{
	if (pts.size() < 3U)
		return false;
	TColgp_Array1OfPnt arr(1, static_cast<int>(pts.size()));
	for (int i = 1; i <= arr.Upper(); ++i)
		arr.SetValue(i, pts[static_cast<std::size_t>(i - 1)]);
	// 三点路径 DegMin=3 常失败，降到 1 仍可得到单边光滑 spine
	for (int degMin : {3, 2, 1})
	{
		GeomAPI_PointsToBSpline fitter(arr, degMin, 8, GeomAbs_C1, 1.0e-3);
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

bool tryMakeBSplineWire(const std::vector<gp_Pnt>& pts, TopoDS_Wire& outWire, bool forceFit)
{
	if (pts.size() < 3U)
		return false;

	if (!forceFit)
	{
		int orthoCorners = 0;
		for (std::size_t i = 2; i < pts.size(); ++i)
		{
			gp_Vec v0(pts[i - 2], pts[i - 1]);
			gp_Vec v1(pts[i - 1], pts[i]);
			if (v0.Magnitude() < kPathEps || v1.Magnitude() < kPathEps)
				continue;
			v0.Normalize();
			v1.Normalize();
			// 仅近直角视为刻意折线；样条三点缓弯不再跳过拟合
			if (std::abs(v0.Dot(v1)) < kOrthoCornerCos)
				++orthoCorners;
		}
		if (pts.size() <= 5U && orthoCorners > 0)
			return false;
	}

	TopoDS_Edge edge;
	if (!fitPointsToBSplineEdge(pts, edge))
		return false;
	BRepBuilderAPI_MakeWire wireMaker(edge);
	if (!wireMaker.IsDone())
		return false;
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
	const std::size_t n = xyz.size() / 3U;
	std::vector<gp_Pnt> pts;
	pts.reserve(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		const gp_Pnt p(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
		if (!pts.empty() && pts.back().Distance(p) < kPathEps)
			continue;
		pts.push_back(p);
	}
	if (pts.size() < 2U)
	{
		if (errMsg)
			*errMsg = "path has no usable edges";
		return false;
	}
	// 折线回退路径同样优先光滑 spine
	if (tryMakeBSplineWire(pts, outWire, pts.size() > 5U))
		return true;
	if (tryMakeBSplineWire(pts, outWire, true))
		return true;

	BRepBuilderAPI_MakeWire wireMaker;
	int edges = 0;
	for (std::size_t i = 1; i < pts.size(); ++i)
	{
		BRepBuilderAPI_MakeEdge edgeMaker(pts[i - 1], pts[i]);
		if (!edgeMaker.IsDone())
		{
			if (errMsg)
				*errMsg = "path MakeEdge failed";
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
	bool hasSpline = false;
	bool hasLine = false;
	for (const auto& s : segs)
	{
		if (s.kind == SketchSweepPathSegKind::Arc)
			hasArc = true;
		else if (s.kind == SketchSweepPathSegKind::SplineThrough)
			hasSpline = true;
		else
			hasLine = true;
	}

	auto collectPts = [&](bool splineOnly) -> std::vector<gp_Pnt>
	{
		std::vector<gp_Pnt> pts;
		pts.reserve(segs.size() + 1U);
		bool first = true;
		for (const auto& s : segs)
		{
			if (splineOnly)
			{
				if (s.kind != SketchSweepPathSegKind::SplineThrough)
					continue;
			}
			else if (s.kind == SketchSweepPathSegKind::Arc || s.kind == SketchSweepPathSegKind::SplineThrough)
			{
				continue;
			}
			const gp_Pnt a(s.ax, s.ay, s.az);
			const gp_Pnt b(s.bx, s.by, s.bz);
			if (first)
			{
				pts.push_back(a);
				first = false;
			}
			else if (pts.back().Distance(a) > kPathEps)
				pts.push_back(a);
			if (pts.back().Distance(b) > kPathEps)
				pts.push_back(b);
		}
		return pts;
	};

	// 含样条：整段过点强制单条 BSpline（含 Line+Spline 混合时仍优先光滑 spine）
	if (hasSpline && !hasArc)
	{
		std::vector<gp_Pnt> pts;
		pts.reserve(segs.size() + 1U);
		bool first = true;
		for (const auto& s : segs)
		{
			if (s.kind == SketchSweepPathSegKind::Arc)
				continue;
			const gp_Pnt a(s.ax, s.ay, s.az);
			const gp_Pnt b(s.bx, s.by, s.bz);
			if (first)
			{
				pts.push_back(a);
				first = false;
			}
			else if (pts.back().Distance(a) > kPathEps)
				pts.push_back(a);
			if (pts.back().Distance(b) > kPathEps)
				pts.push_back(b);
		}
		if (tryMakeBSplineWire(pts, outWire, true))
			return true;
		if (!hasLine)
		{
			if (errMsg)
				*errMsg = "spline path BSpline fit failed";
			return false;
		}
	}
	// 纯折线：非 L 形一律强制拟合（旧版样条采样常落成 Line）
	if (!hasArc && !hasSpline)
	{
		const std::vector<gp_Pnt> pts = collectPts(false);
		bool force = true;
		if (pts.size() <= 5U)
		{
			int ortho = 0;
			for (std::size_t i = 2; i < pts.size(); ++i)
			{
				gp_Vec v0(pts[i - 2], pts[i - 1]);
				gp_Vec v1(pts[i - 1], pts[i]);
				if (v0.Magnitude() < kPathEps || v1.Magnitude() < kPathEps)
					continue;
				v0.Normalize();
				v1.Normalize();
				if (std::abs(v0.Dot(v1)) < kOrthoCornerCos)
					++ortho;
			}
			force = (ortho == 0);
		}
		if (tryMakeBSplineWire(pts, outWire, force))
			return true;
	}

	BRepBuilderAPI_MakeWire wireMaker;
	int edges = 0;
	std::vector<gp_Pnt> splineRun;
	auto flushSplineRun = [&]() -> bool
	{
		if (splineRun.size() < 2U)
		{
			splineRun.clear();
			return true;
		}
		TopoDS_Wire sw;
		if (splineRun.size() >= 3U && tryMakeBSplineWire(splineRun, sw, true))
		{
			for (TopExp_Explorer ex(sw, TopAbs_EDGE); ex.More(); ex.Next())
			{
				wireMaker.Add(TopoDS::Edge(ex.Current()));
				if (!wireMaker.IsDone())
					return false;
				++edges;
			}
			splineRun.clear();
			return true;
		}
		for (std::size_t i = 1; i < splineRun.size(); ++i)
		{
			BRepBuilderAPI_MakeEdge edgeMaker(splineRun[i - 1], splineRun[i]);
			if (!edgeMaker.IsDone())
				return false;
			wireMaker.Add(edgeMaker.Edge());
			if (!wireMaker.IsDone())
				return false;
			++edges;
		}
		splineRun.clear();
		return true;
	};

	for (const auto& s : segs)
	{
		const gp_Pnt a(s.ax, s.ay, s.az);
		const gp_Pnt b(s.bx, s.by, s.bz);
		if (s.kind == SketchSweepPathSegKind::SplineThrough)
		{
			if (splineRun.empty())
				splineRun.push_back(a);
			else if (splineRun.back().Distance(a) > kPathEps)
				splineRun.push_back(a);
			if (splineRun.back().Distance(b) > kPathEps)
				splineRun.push_back(b);
			continue;
		}
		if (!flushSplineRun())
		{
			if (errMsg)
				*errMsg = "spline run flush failed";
			return false;
		}
		TopoDS_Edge edge;
		if (s.kind == SketchSweepPathSegKind::Arc)
		{
			const gp_Pnt m(s.mx, s.my, s.mz);
			GC_MakeArcOfCircle mkArc(a, m, b);
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
			edge = edgeMaker.Edge();
		}
		else
		{
			if (a.Distance(b) < kPathEps)
				continue;
			BRepBuilderAPI_MakeEdge edgeMaker(a, b);
			if (!edgeMaker.IsDone())
			{
				if (errMsg)
					*errMsg = "path MakeEdge failed";
				return false;
			}
			edge = edgeMaker.Edge();
		}
		wireMaker.Add(edge);
		if (!wireMaker.IsDone())
		{
			if (errMsg)
				*errMsg = "path edges do not form a wire";
			return false;
		}
		++edges;
	}
	if (!flushSplineRun())
	{
		if (errMsg)
			*errMsg = "spline run flush failed";
		return false;
	}
	if (edges == 0 && wireMaker.IsDone())
	{
		outWire = wireMaker.Wire();
		return !outWire.IsNull();
	}
	if (edges == 0)
	{
		if (errMsg)
			*errMsg = "path has no usable edges";
		return false;
	}
	outWire = wireMaker.Wire();
	if (outWire.IsNull())
		return false;
	// 多边 spine 会把侧壁切成平面条带；无圆弧时再强制合成单条 BSpline
	if (!hasArc)
	{
		int edgeCount = 0;
		for (TopExp_Explorer ex(outWire, TopAbs_EDGE); ex.More(); ex.Next())
			++edgeCount;
		if (edgeCount > 1)
		{
			std::vector<gp_Pnt> densified;
			for (TopExp_Explorer ex(outWire, TopAbs_EDGE); ex.More(); ex.Next())
			{
				BRepAdaptor_Curve c(TopoDS::Edge(ex.Current()));
				constexpr int n = 8;
				for (int i = 0; i <= n; ++i)
				{
					const double u =
						c.FirstParameter() + (c.LastParameter() - c.FirstParameter()) * static_cast<double>(i) / n;
					const gp_Pnt p = c.Value(u);
					if (densified.empty() || densified.back().Distance(p) > kPathEps)
						densified.push_back(p);
				}
			}
			TopoDS_Wire single;
			if (tryMakeBSplineWire(densified, single, true))
				outWire = single;
		}
	}
	return !outWire.IsNull();
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
			const gp_Pnt p = c.Value(u);
			if (pts.empty() || pts.back().Distance(p) > 1e-9)
				pts.push_back(p);
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

bool shapeLooksLikeFacetedSweep(const TopoDS_Shape& shape)
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
	// 正常矩形扫描约 6 面；碎成平面条带时面数暴涨且几乎全是平面
	return faces > 24 && planar * 2 >= faces;
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

		auto accept = [&](const TopoDS_Shape& s) -> bool
		{
			if (s.IsNull() || shapeLooksLikeFacetedSweep(s))
				return false;
			outTool = s;
			return true;
		};

		// OneCAD：MakePipe(spine, profileWire)；优先 Face 以得实体
		{
			BRepOffsetAPI_MakePipe pipe(pathWire, profileFace);
			if (pipe.IsDone() && accept(pipe.Shape()))
				return true;
		}
		{
			BRepOffsetAPI_MakePipe pipe(pathWire, profileWire);
			if (pipe.IsDone() && accept(pipe.Shape()))
				return true;
		}
		{
			BRepOffsetAPI_MakePipe pipe(pathWire, profileFace, GeomFill_IsCorrectedFrenet, Standard_True);
			if (pipe.IsDone() && accept(pipe.Shape()))
				return true;
		}

		// 平面路径恒定副法线；仅当前述失败时使用（避免伪造成功的碎面优先返回）
		{
			BRepOffsetAPI_MakePipeShell shell(pathWire);
			shell.SetTolerance(1.0e-4, 1.0e-4, 1.0e-2);
			gp_Dir biNormal;
			if (tryPlanarPathBinormal(pathWire, biNormal))
				shell.SetMode(biNormal);
			else
				shell.SetMode(Standard_False);
			shell.Add(profileWire, Standard_False, Standard_False);
			if (shell.IsReady())
			{
				shell.Build();
				if (shell.IsDone())
				{
					(void)shell.MakeSolid();
					if (accept(shell.Shape()))
						return true;
				}
			}
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

bool pipeAndBoolean(const TopoDS_Face& profileFace, const TopoDS_Wire& pathWire, const SketchSweepParams& params,
					const ShapeHandle* baseOrNull, ShapeHandle& outShape, std::string* errMsg)
{
	TopoDS_Shape tool;
	if (!buildPipeSolid(profileFace, pathWire, tool, errMsg))
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
		{
			result = tool;
		}
		else if (!brepBooleanToShape(*basePtr, tool, BrepBooleanOp::Fuse, result, errMsg))
		{
			return false;
		}
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
	if (!makeClosedFace(profilePolylineXyzMm, profileFace, errMsg))
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
	if (!makeClosedFace(profilePolylineXyzMm, profileFace, errMsg))
		return false;
	TopoDS_Wire pathWire;
	if (!makePathWireFromSegments(pathSegments, pathWire, errMsg))
		return false;
	return pipeAndBoolean(profileFace, pathWire, params, baseOrNull, outShape, errMsg);
}

} // namespace geoalgo
