/// @file HlrProject.cpp
/// @brief OCC HLR / 剖切投影为图面折线

#include "HlrProject.h"

#include "Discretize.h"
#include "DrawingEngines.h"
#include "DrawingGeometry.h"
#include "ShapeHandle.h"

#include <BRepAlgoAPI_Section.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <gp_Ax2.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <cmath>
#include <future>
#include <unordered_set>

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
		return third ? gp_Ax2(center, gp_Dir(0.0, 0.0, 1.0), gp_Dir(1.0, 0.0, 0.0))
					 : gp_Ax2(center, gp_Dir(0.0, 0.0, -1.0), gp_Dir(1.0, 0.0, 0.0));
	case HlrViewKind::Right:
		return third ? gp_Ax2(center, gp_Dir(-1.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0))
					 : gp_Ax2(center, gp_Dir(1.0, 0.0, 0.0), gp_Dir(0.0, 1.0, 0.0));
	case HlrViewKind::Iso:
	{
		const double invSqrt3 = 1.0 / std::sqrt(3.0);
		const gp_Dir view(invSqrt3, invSqrt3, invSqrt3);
		const gp_Dir xDir(-1.0, 1.0, 0.0);
		return gp_Ax2(center, view, xDir);
	}
	case HlrViewKind::Front:
	default:
		return third ? gp_Ax2(center, gp_Dir(0.0, 1.0, 0.0), gp_Dir(1.0, 0.0, 0.0))
					 : gp_Ax2(center, gp_Dir(0.0, -1.0, 0.0), gp_Dir(1.0, 0.0, 0.0));
	}
}

TopoDS_Shape unifySameDomainOnce(const TopoDS_Shape& native)
{
	TopoDS_Shape hlrInput = native;
	ShapeUpgrade_UnifySameDomain unify(native, true, true, false);
	unify.Build();
	const TopoDS_Shape u = unify.Shape();
	if (!u.IsNull())
		hlrInput = u;
	return hlrInput;
}

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

double polyLen2d(const Polyline3d& p)
{
	double len = 0;
	for (std::size_t i = 3; i + 2 < p.xyz.size(); i += 3)
	{
		const double dx = p.xyz[i] - p.xyz[i - 3];
		const double dy = p.xyz[i + 1] - p.xyz[i - 2];
		len += std::sqrt(dx * dx + dy * dy);
	}
	return len;
}

bool polylinesNearlyEqual2d(const Polyline3d& a, const Polyline3d& b, double tol)
{
	if (a.xyz.size() < 6 || b.xyz.size() < 6)
		return false;
	const auto endPts = [](const Polyline3d& p, float& x0, float& y0, float& x1, float& y1) {
		x0 = p.xyz[0];
		y0 = p.xyz[1];
		x1 = p.xyz[p.xyz.size() - 3];
		y1 = p.xyz[p.xyz.size() - 2];
	};
	float ax0, ay0, ax1, ay1, bx0, by0, bx1, by1;
	endPts(a, ax0, ay0, ax1, ay1);
	endPts(b, bx0, by0, bx1, by1);
	const auto d2 = [](float x0, float y0, float x1, float y1) {
		const double dx = x0 - x1, dy = y0 - y1;
		return dx * dx + dy * dy;
	};
	const double tol2 = tol * tol;
	const bool sameDir = d2(ax0, ay0, bx0, by0) <= tol2 && d2(ax1, ay1, bx1, by1) <= tol2;
	const bool revDir = d2(ax0, ay0, bx1, by1) <= tol2 && d2(ax1, ay1, bx0, by0) <= tol2;
	if (!sameDir && !revDir)
		return false;
	const double la = polyLen2d(a), lb = polyLen2d(b);
	if (la < 1e-9 || lb < 1e-9)
		return true;
	return std::fabs(la - lb) <= tol * 4.0 || std::fabs(la - lb) / (std::max)(la, lb) < 0.08;
}

/// 递归抽稀（DP）：把近共线点合并，生成阶段一次完成
void douglasPeucker2d(const std::vector<float>& xyz, std::size_t i0, std::size_t i1, double eps2, std::vector<char>& keep)
{
	if (i1 <= i0 + 1)
		return;
	const double x0 = xyz[i0 * 3], y0 = xyz[i0 * 3 + 1];
	const double x1 = xyz[i1 * 3], y1 = xyz[i1 * 3 + 1];
	const double dx = x1 - x0, dy = y1 - y0;
	const double len2 = dx * dx + dy * dy;
	double maxD2 = -1.0;
	std::size_t maxI = i0;
	for (std::size_t i = i0 + 1; i < i1; ++i)
	{
		const double px = xyz[i * 3], py = xyz[i * 3 + 1];
		double d2 = 0;
		if (len2 < 1e-18)
		{
			const double ddx = px - x0, ddy = py - y0;
			d2 = ddx * ddx + ddy * ddy;
		}
		else
		{
			const double t = ((px - x0) * dx + (py - y0) * dy) / len2;
			const double qx = x0 + t * dx, qy = y0 + t * dy;
			const double ddx = px - qx, ddy = py - qy;
			d2 = ddx * ddx + ddy * ddy;
		}
		if (d2 > maxD2)
		{
			maxD2 = d2;
			maxI = i;
		}
	}
	if (maxD2 <= eps2)
		return;
	keep[maxI] = 1;
	douglasPeucker2d(xyz, i0, maxI, eps2, keep);
	douglasPeucker2d(xyz, maxI, i1, eps2, keep);
}

/// 单条折线抽稀：闭合首尾都保留
void simplifyPolyline2d(Polyline3d& poly, double epsMm)
{
	const std::size_t n = poly.xyz.size() / 3;
	if (n < 4)
		return;
	std::vector<char> keep(n, 0);
	keep[0] = 1;
	keep[n - 1] = 1;
	douglasPeucker2d(poly.xyz, 0, n - 1, epsMm * epsMm, keep);
	std::vector<float> out;
	out.reserve(n * 3);
	for (std::size_t i = 0; i < n; ++i)
	{
		if (!keep[i])
			continue;
		out.push_back(poly.xyz[i * 3]);
		out.push_back(poly.xyz[i * 3 + 1]);
		out.push_back(poly.xyz[i * 3 + 2]);
	}
	if (out.size() >= 6)
		poly.xyz.swap(out);
}

void simplifyDrawingPolylines(std::vector<Polyline3d>& polys, double epsMm)
{
	for (Polyline3d& poly : polys)
		simplifyPolyline2d(poly, epsMm);
}

/// 可见/隐藏复合体偶发重复边，叠画会把薄件糊黑；大模型用端点哈希近似去重，避免 O(n²)
void dedupeDrawingPolylines(std::vector<Polyline3d>& polys, double tolMm)
{
	if (polys.size() < 2)
		return;
	const std::size_t n = polys.size();
	// 规模较大时跳过逐对长度比对，仅按端点哈希去重
	const bool heavy = n > 1500;
	std::vector<char> drop(n, 0);
	if (heavy)
	{
		std::unordered_set<long long> seen;
		seen.reserve(n);
		const double inv = 1.0 / (tolMm > 1e-6 ? tolMm : 1e-6);
		for (std::size_t i = 0; i < n; ++i)
		{
			const Polyline3d& p = polys[i];
			if (p.xyz.size() < 6)
			{
				drop[i] = 1;
				continue;
			}
			auto q = [&](float v) { return static_cast<int>(std::lround(v * inv)); };
			const long long key0 = (static_cast<long long>(q(p.xyz[0])) << 32) |
								   static_cast<unsigned int>(q(p.xyz[1]));
			const long long key1 = (static_cast<long long>(q(p.xyz[p.xyz.size() - 3])) << 32) |
								   static_cast<unsigned int>(q(p.xyz[p.xyz.size() - 2]));
			const long long key = key0 ^ (key1 << 1);
			if (seen.find(key) != seen.end())
			{
				drop[i] = 1;
				continue;
			}
			seen.insert(key);
		}
	}
	else
	{
		for (std::size_t i = 0; i < n; ++i)
		{
			if (drop[i])
				continue;
			for (std::size_t j = i + 1; j < n; ++j)
			{
				if (drop[j])
					continue;
				if (polylinesNearlyEqual2d(polys[i], polys[j], tolMm))
					drop[j] = 1;
			}
		}
	}
	std::vector<Polyline3d> kept;
	kept.reserve(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		if (!drop[i])
			kept.push_back(std::move(polys[i]));
	}
	polys.swap(kept);
}

void removeHiddenCoveredByVisible(std::vector<Polyline3d>& hidden, const std::vector<Polyline3d>& visible,
								  double tolMm)
{
	if (hidden.empty() || visible.empty())
		return;
	const double inv = 1.0 / (tolMm > 1e-6 ? tolMm : 1e-6);
	auto endKey = [&](const Polyline3d& p) -> long long {
		if (p.xyz.size() < 6)
			return 0;
		auto q = [&](float v) { return static_cast<int>(std::lround(v * inv)); };
		long long a = (static_cast<long long>(q(p.xyz[0])) << 32) | static_cast<unsigned int>(q(p.xyz[1]));
		long long b = (static_cast<long long>(q(p.xyz[p.xyz.size() - 3])) << 32) |
					  static_cast<unsigned int>(q(p.xyz[p.xyz.size() - 2]));
		if (a > b)
			std::swap(a, b);
		return a ^ (b << 1);
	};
	std::unordered_set<long long> visKeys;
	visKeys.reserve(visible.size() * 2);
	for (const Polyline3d& v : visible)
	{
		if (v.xyz.size() >= 6)
			visKeys.insert(endKey(v));
	}
	std::vector<Polyline3d> kept;
	kept.reserve(hidden.size());
	if (hidden.size() + visible.size() > 2000)
	{
		// 大模型用端点键剔除与可见重合的隐藏边，避免糊成双线
		for (Polyline3d& h : hidden)
		{
			if (h.xyz.size() < 6)
				continue;
			if (visKeys.find(endKey(h)) != visKeys.end())
				continue;
			kept.push_back(std::move(h));
		}
		hidden.swap(kept);
		return;
	}
	for (Polyline3d& h : hidden)
	{
		bool covered = visKeys.find(endKey(h)) != visKeys.end();
		if (!covered)
		{
			for (const Polyline3d& v : visible)
			{
				if (polylinesNearlyEqual2d(h, v, tolMm))
				{
					covered = true;
					break;
				}
			}
		}
		if (!covered)
			kept.push_back(std::move(h));
	}
	hidden.swap(kept);
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

/// 入参已 Unify；引擎产出图元 → 折线后再 sanitize/DP/dedupe
bool projectShapeHlr(const TopoDS_Shape& nativeShapeAlreadyUnified, HlrViewKind kind, HlrProjectionAngle angle,
					 const TessellateParams& params, const DrawingHlrRunOptions& options, HlrViewPolylines& out,
					 std::string* errMsg)
{
	out = HlrViewPolylines{};
	if (nativeShapeAlreadyUnified.IsNull())
	{
		if (errMsg)
			*errMsg = "HLR: null shape";
		return false;
	}

	gp_Pnt center;
	Bnd_Box box;
	if (!shapeCenterAndBox(nativeShapeAlreadyUnified, center, box, errMsg))
		return false;

	const gp_Ax2 viewAx = projectorAx2(kind, angle, center);
	const double diag = shapeDiagonal(box);

	std::vector<DrawingEntity> ents;
	ents.reserve(256);
	std::string engineErr;
	bool ok = false;
	if (options.useMeshHlr)
	{
		ok = drawing_engines::extractMeshHlrEntities(nativeShapeAlreadyUnified, viewAx, params, ents, &engineErr);
		// 网格预览失败时回落精确，避免空视图
		if (!ok)
			ok = drawing_engines::extractExactHlrEntities(nativeShapeAlreadyUnified, viewAx, options.nbIso, params,
														 ents, &engineErr);
	}
	else
	{
		ok = drawing_engines::extractExactHlrEntities(nativeShapeAlreadyUnified, viewAx, options.nbIso, params, ents,
													 &engineErr);
	}
	if (!ok)
	{
		if (errMsg)
			*errMsg = engineErr.empty() ? "HLR: no projection edges" : engineErr;
		return false;
	}

	drawingEntitiesToPolylines(ents, out.visible, out.hidden);

	const double limit = (std::max)(diag * 50.0, 1000.0);
	sanitizeDrawingPolylines(out.visible, limit);
	sanitizeDrawingPolylines(out.hidden, limit);
	// 生成侧一次抽稀：去掉近共线密集采样点，交互只做平移
	const double simpEps = (std::max)(0.03, diag * 6e-5);
	simplifyDrawingPolylines(out.visible, simpEps);
	simplifyDrawingPolylines(out.hidden, simpEps);
	const double dedupeTol = (std::max)(0.05, diag * 2e-4);
	dedupeDrawingPolylines(out.visible, dedupeTol);
	dedupeDrawingPolylines(out.hidden, dedupeTol);
	removeHiddenCoveredByVisible(out.hidden, out.visible, dedupeTol);

	if (out.visible.empty() && out.hidden.empty())
	{
		if (errMsg)
			*errMsg = "HLR: no projection edges";
		return false;
	}
	return true;
}

bool projectShapeHlr(const TopoDS_Shape& nativeShapeAlreadyUnified, HlrViewKind kind, HlrProjectionAngle angle,
					 const TessellateParams& params, HlrViewPolylines& out, std::string* errMsg)
{
	return projectShapeHlr(nativeShapeAlreadyUnified, kind, angle, params, DrawingHlrRunOptions{}, out, errMsg);
}

} // namespace

bool projectShapeHlr(const ShapeHandle& shape, HlrViewKind kind, HlrProjectionAngle angle,
					 const TessellateParams& params, const DrawingHlrRunOptions& options, HlrViewPolylines& out,
					 std::string* errMsg)
{
	out = HlrViewPolylines{};
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		if (errMsg)
			*errMsg = "HLR: null ShapeHandle";
		return false;
	}
	return projectShapeHlr(unifySameDomainOnce(native), kind, angle, params, options, out, errMsg);
}

bool projectShapeHlr(const ShapeHandle& shape, HlrViewKind kind, HlrProjectionAngle angle,
					 const TessellateParams& params, HlrViewPolylines& out, std::string* errMsg)
{
	return projectShapeHlr(shape, kind, angle, params, DrawingHlrRunOptions{}, out, errMsg);
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
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		if (errMsg)
			*errMsg = "HLR: null ShapeHandle";
		return false;
	}
	const TopoDS_Shape unified = unifySameDomainOnce(native);
	const DrawingHlrRunOptions opts{};
	// 各视图独立 HLR 实例，可并行
	std::future<bool> fFront = std::async(std::launch::async, [&]() {
		return projectShapeHlr(unified, HlrViewKind::Front, angle, params, opts, out.front, nullptr);
	});
	std::future<bool> fTop = std::async(std::launch::async, [&]() {
		return projectShapeHlr(unified, HlrViewKind::Top, angle, params, opts, out.top, nullptr);
	});
	std::future<bool> fRight = std::async(std::launch::async, [&]() {
		return projectShapeHlr(unified, HlrViewKind::Right, angle, params, opts, out.right, nullptr);
	});
	const bool okF = fFront.get();
	const bool okT = fTop.get();
	const bool okR = fRight.get();
	if (!okF || !okT || !okR)
	{
		if (errMsg)
			*errMsg = "HLR: three-views projection failed";
		return false;
	}
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
										nullptr, params, DrawingHlrRunOptions{}, out, errMsg);
}

bool projectShapeHlrDrawingBundle(const ShapeHandle& shape, HlrProjectionAngle angle, bool includeIso,
								  bool includeSection, DrawingSectionPlane sectionPlane, bool customSection,
								  const double originMm[3], const double normal[3], const TessellateParams& params,
								  HlrDrawingBundle& out, std::string* errMsg)
{
	return projectShapeHlrDrawingBundle(shape, angle, includeIso, includeSection, sectionPlane, customSection,
										originMm, normal, params, DrawingHlrRunOptions{}, out, errMsg);
}

bool projectShapeHlrDrawingBundle(const ShapeHandle& shape, HlrProjectionAngle angle, bool includeIso,
								  bool includeSection, DrawingSectionPlane sectionPlane, bool customSection,
								  const double originMm[3], const double normal[3], const TessellateParams& params,
								  const DrawingHlrRunOptions& options, HlrDrawingBundle& out, std::string* errMsg)
{
	out = HlrDrawingBundle{};
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		if (errMsg)
			*errMsg = "HLR: null ShapeHandle";
		return false;
	}
	// 整包只 Unify 一次；各视图独立 Algo 实例并行
	// 网格仅 coarseView 显式开启；勿按边数自动降级，否则正式出图圆/弧呈多边形多线
	const TopoDS_Shape unified = unifySameDomainOnce(native);
	const DrawingHlrRunOptions runOpts = options;

	std::future<bool> fFront = std::async(std::launch::async, [&]() {
		return projectShapeHlr(unified, HlrViewKind::Front, angle, params, runOpts, out.front, nullptr);
	});
	std::future<bool> fTop = std::async(std::launch::async, [&]() {
		return projectShapeHlr(unified, HlrViewKind::Top, angle, params, runOpts, out.top, nullptr);
	});
	std::future<bool> fRight = std::async(std::launch::async, [&]() {
		return projectShapeHlr(unified, HlrViewKind::Right, angle, params, runOpts, out.right, nullptr);
	});
	std::future<bool> fIso;
	if (includeIso)
	{
		fIso = std::async(std::launch::async, [&]() {
			return projectShapeHlr(unified, HlrViewKind::Iso, angle, params, runOpts, out.iso, nullptr);
		});
	}
	std::future<bool> fSec;
	if (includeSection)
	{
		fSec = std::async(std::launch::async, [&]() {
			if (customSection && originMm && normal)
				return sectionShapeToDrawing(shape, originMm, normal, params, out.section, nullptr);
			return sectionShapeToDrawing(shape, sectionPlane, params, out.section, nullptr);
		});
	}

	const bool okF = fFront.get();
	const bool okT = fTop.get();
	const bool okR = fRight.get();
	if (!okF || !okT || !okR)
	{
		if (errMsg)
			*errMsg = "HLR: orthographic projection failed";
		return false;
	}
	if (includeIso && fIso.valid())
		out.hasIso = fIso.get();
	if (includeSection && fSec.valid())
		out.hasSection = fSec.get();
	return true;
}

} // namespace geoalgo
