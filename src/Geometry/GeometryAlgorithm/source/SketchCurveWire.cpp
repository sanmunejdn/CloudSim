/// @file SketchCurveWire.cpp
/// @brief 近圆/近椭圆提升 + Line/Arc/Circle/Ellipse 真曲线 wire

#include "detail/SketchCurveWireOcc.h"

#include "detail/OccIncludes.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Geom_Ellipse.hxx>
#include <gp_Ax2.hxx>
#include <gp_Circ.hxx>
#include <gp_Dir.hxx>
#include <gp_Elips.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>

namespace geoalgo
{
namespace
{
constexpr double kEps = 1e-9;

bool tryMakeCircularWire(const std::vector<float>& xyz, const gp_Dir& planeNormal, TopoDS_Wire& outWire)
{
	if (xyz.size() < 24U || (xyz.size() % 3U) != 0U)
		return false;
	std::size_t n = xyz.size() / 3U;
	const gp_Pnt p0(xyz[0], xyz[1], xyz[2]);
	const gp_Pnt pLast(xyz[(n - 1) * 3], xyz[(n - 1) * 3 + 1], xyz[(n - 1) * 3 + 2]);
	if (p0.Distance(pLast) <= 1e-6 && n > 1)
		--n;
	if (n < 8)
		return false;

	double cx = 0, cy = 0, cz = 0;
	for (std::size_t i = 0; i < n; ++i)
	{
		cx += xyz[i * 3];
		cy += xyz[i * 3 + 1];
		cz += xyz[i * 3 + 2];
	}
	cx /= static_cast<double>(n);
	cy /= static_cast<double>(n);
	cz /= static_cast<double>(n);
	const gp_Pnt center(cx, cy, cz);

	double rMin = 1e300, rMax = 0.0, rSum = 0.0;
	for (std::size_t i = 0; i < n; ++i)
	{
		const double r = gp_Pnt(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]).Distance(center);
		rMin = std::min(rMin, r);
		rMax = std::max(rMax, r);
		rSum += r;
	}
	const double rMean = rSum / static_cast<double>(n);
	if (rMean < 1e-6 || (rMax - rMin) > std::max(0.05, 0.02 * rMean))
		return false;

	double planeDev = 0.0;
	for (std::size_t i = 0; i < n; ++i)
	{
		const gp_Pnt p(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]);
		planeDev = std::max(planeDev, std::abs(gp_Vec(center, p).Dot(gp_Vec(planeNormal))));
	}
	if (planeDev > std::max(0.05, 0.01 * rMean))
		return false;

	try
	{
		BRepBuilderAPI_MakeEdge mkEdge(gp_Circ(gp_Ax2(center, planeNormal), rMean));
		if (!mkEdge.IsDone())
			return false;
		BRepBuilderAPI_MakeWire mkWire(mkEdge.Edge());
		if (!mkWire.IsDone())
			return false;
		outWire = mkWire.Wire();
		return !outWire.IsNull();
	}
	catch (...)
	{
		return false;
	}
}

bool tryMakeEllipticalWire(const std::vector<float>& xyz, const gp_Dir& planeNormal, TopoDS_Wire& outWire)
{
	if (xyz.size() < 36U || (xyz.size() % 3U) != 0U)
		return false;
	std::size_t n = xyz.size() / 3U;
	const gp_Pnt p0(xyz[0], xyz[1], xyz[2]);
	const gp_Pnt pLast(xyz[(n - 1) * 3], xyz[(n - 1) * 3 + 1], xyz[(n - 1) * 3 + 2]);
	if (p0.Distance(pLast) <= 1e-6 && n > 1)
		--n;
	if (n < 12)
		return false;

	double cx = 0, cy = 0, cz = 0;
	for (std::size_t i = 0; i < n; ++i)
	{
		cx += xyz[i * 3];
		cy += xyz[i * 3 + 1];
		cz += xyz[i * 3 + 2];
	}
	cx /= static_cast<double>(n);
	cy /= static_cast<double>(n);
	cz /= static_cast<double>(n);
	const gp_Pnt center(cx, cy, cz);

	gp_Dir xHint = planeNormal.X() * planeNormal.X() < 0.9 ? gp_Dir(1, 0, 0) : gp_Dir(0, 1, 0);
	gp_Vec xAxis = gp_Vec(xHint).Crossed(gp_Vec(planeNormal));
	if (xAxis.Magnitude() < 1e-12)
		return false;
	xAxis.Normalize();
	gp_Vec yAxis = gp_Vec(planeNormal).Crossed(xAxis);
	yAxis.Normalize();

	double sumUU = 0, sumVV = 0, sumUV = 0;
	std::vector<double> uu(n), vv(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		const gp_Vec d(center, gp_Pnt(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
		uu[i] = d.Dot(xAxis);
		vv[i] = d.Dot(yAxis);
		sumUU += uu[i] * uu[i];
		sumVV += vv[i] * vv[i];
		sumUV += uu[i] * vv[i];
	}
	const double a = sumUU / static_cast<double>(n);
	const double b = sumUV / static_cast<double>(n);
	const double c = sumVV / static_cast<double>(n);
	const double disc = std::sqrt(std::max(0.0, (a - c) * (a - c) + 4 * b * b));
	const double l1 = 0.5 * (a + c + disc);
	const double l2 = 0.5 * (a + c - disc);
	if (l1 < 1e-12 || l2 < 1e-12)
		return false;
	const double majorR = std::sqrt(l1);
	const double minorR = std::sqrt(l2);
	if (majorR < 1e-6 || minorR < 1e-6)
		return false;
	if (std::abs(majorR - minorR) / majorR < 0.02)
		return false; // 近圆交给 circle

	double ang = 0.5 * std::atan2(2 * b, a - c);
	gp_Vec majDir = xAxis * std::cos(ang) + yAxis * std::sin(ang);
	if (majDir.Magnitude() < 1e-12)
		return false;
	majDir.Normalize();

	double maxErr = 0.0;
	for (std::size_t i = 0; i < n; ++i)
	{
		const double u = uu[i] * std::cos(ang) + vv[i] * std::sin(ang);
		const double v = -uu[i] * std::sin(ang) + vv[i] * std::cos(ang);
		const double e = (u * u) / (majorR * majorR) + (v * v) / (minorR * minorR);
		maxErr = std::max(maxErr, std::abs(e - 1.0));
	}
	if (maxErr > 0.05)
		return false;

	try
	{
		const gp_Ax2 ax(center, planeNormal, gp_Dir(majDir));
		const gp_Elips el(ax, majorR, minorR);
		BRepBuilderAPI_MakeEdge mkEdge(el);
		if (!mkEdge.IsDone())
			return false;
		BRepBuilderAPI_MakeWire mkWire(mkEdge.Edge());
		if (!mkWire.IsDone())
			return false;
		outWire = mkWire.Wire();
		return !outWire.IsNull();
	}
	catch (...)
	{
		return false;
	}
}

bool makePolygonWire(const std::vector<float>& xyz, TopoDS_Wire& outWire, std::string* errMsg)
{
	if (xyz.size() < 9U || (xyz.size() % 3U) != 0U)
	{
		if (errMsg)
			*errMsg = "polyline needs >=3 points";
		return false;
	}
	BRepBuilderAPI_MakePolygon poly;
	const std::size_t n = xyz.size() / 3U;
	for (std::size_t i = 0; i < n; ++i)
		poly.Add(gp_Pnt(xyz[i * 3], xyz[i * 3 + 1], xyz[i * 3 + 2]));
	const gp_Pnt p0(xyz[0], xyz[1], xyz[2]);
	const gp_Pnt pLast(xyz[(n - 1) * 3], xyz[(n - 1) * 3 + 1], xyz[(n - 1) * 3 + 2]);
	if (p0.Distance(pLast) > 1e-6)
		poly.Add(p0);
	poly.Close();
	if (!poly.IsDone())
	{
		if (errMsg)
			*errMsg = "MakePolygon failed";
		return false;
	}
	outWire = poly.Wire();
	return !outWire.IsNull();
}

gp_Dir resolveNormal(double nx, double ny, double nz, const gp_Dir* fallback)
{
	const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
	if (len > 1e-12)
		return gp_Dir(nx / len, ny / len, nz / len);
	if (fallback)
		return *fallback;
	return gp_Dir(0, 0, 1);
}
} // namespace

bool estimatePolylinePlaneNormal(const std::vector<float>& xyzMm, double& nx, double& ny, double& nz)
{
	nx = 0;
	ny = 0;
	nz = 1;
	if (xyzMm.size() < 9)
		return false;
	const std::size_t n = xyzMm.size() / 3;
	gp_XYZ acc(0, 0, 0);
	const gp_Pnt p0(xyzMm[0], xyzMm[1], xyzMm[2]);
	for (std::size_t i = 1; i + 1 < n; ++i)
	{
		const gp_Pnt a(xyzMm[i * 3], xyzMm[i * 3 + 1], xyzMm[i * 3 + 2]);
		const gp_Pnt b(xyzMm[(i + 1) * 3], xyzMm[(i + 1) * 3 + 1], xyzMm[(i + 1) * 3 + 2]);
		acc += gp_Vec(p0, a).Crossed(gp_Vec(p0, b)).XYZ();
	}
	if (acc.Modulus() < 1e-12)
		return false;
	acc.Normalize();
	nx = acc.X();
	ny = acc.Y();
	nz = acc.Z();
	return true;
}

bool makeClosedWireFromPolylineMm(const std::vector<float>& xyzMm, double planeNx, double planeNy, double planeNz,
								  TopoDS_Wire& outWire, std::string* errMsg)
{
	double nx = planeNx, ny = planeNy, nz = planeNz;
	if (std::sqrt(nx * nx + ny * ny + nz * nz) < 1e-12)
		(void)estimatePolylinePlaneNormal(xyzMm, nx, ny, nz);
	const gp_Dir n = resolveNormal(nx, ny, nz, nullptr);
	if (tryMakeCircularWire(xyzMm, n, outWire))
		return true;
	if (tryMakeEllipticalWire(xyzMm, n, outWire))
		return true;
	return makePolygonWire(xyzMm, outWire, errMsg);
}

bool makeClosedFaceFromPolylineMm(const std::vector<float>& xyzMm, double planeNx, double planeNy, double planeNz,
								  TopoDS_Face& outFace, std::string* errMsg)
{
	TopoDS_Wire wire;
	if (!makeClosedWireFromPolylineMm(xyzMm, planeNx, planeNy, planeNz, wire, errMsg))
		return false;
	BRepBuilderAPI_MakeFace mkFace(wire, Standard_True);
	if (!mkFace.IsDone())
	{
		if (errMsg)
			*errMsg = "MakeFace from wire failed";
		return false;
	}
	outFace = mkFace.Face();
	return true;
}

bool makeClosedWireFromSegments(const std::vector<SketchCurveSegment>& segs, double planeNx, double planeNy,
								double planeNz, TopoDS_Wire& outWire, std::string* errMsg)
{
	if (segs.empty())
	{
		if (errMsg)
			*errMsg = "profile segments empty";
		return false;
	}

	const gp_Dir planeN = resolveNormal(planeNx, planeNy, planeNz, nullptr);

	if (segs.size() == 1 && segs[0].kind == SketchCurveSegKind::Circle)
	{
		const auto& s = segs[0];
		gp_Dir n = planeN;
		const double nlen = std::sqrt(s.mx * s.mx + s.my * s.my + s.mz * s.mz);
		if (nlen > 1e-12)
			n = gp_Dir(s.mx / nlen, s.my / nlen, s.mz / nlen);
		if (s.bx <= 1e-9)
		{
			if (errMsg)
				*errMsg = "circle radius invalid";
			return false;
		}
		try
		{
			BRepBuilderAPI_MakeEdge mkEdge(gp_Circ(gp_Ax2(gp_Pnt(s.ax, s.ay, s.az), n), s.bx));
			if (!mkEdge.IsDone())
				return false;
			BRepBuilderAPI_MakeWire mkWire(mkEdge.Edge());
			if (!mkWire.IsDone())
				return false;
			outWire = mkWire.Wire();
			return !outWire.IsNull();
		}
		catch (...)
		{
			if (errMsg)
				*errMsg = "circle edge failed";
			return false;
		}
	}

	if (segs.size() == 1 && segs[0].kind == SketchCurveSegKind::Ellipse)
	{
		const auto& s = segs[0];
		gp_Dir n = planeN;
		const double nlen = std::sqrt(s.mx * s.mx + s.my * s.my + s.mz * s.mz);
		if (nlen > 1e-12)
			n = gp_Dir(s.mx / nlen, s.my / nlen, s.mz / nlen);
		const double majorR = s.bx;
		const double minorR = s.by;
		if (majorR <= 1e-9 || minorR <= 1e-9 || majorR < minorR)
		{
			if (errMsg)
				*errMsg = "ellipse radii invalid";
			return false;
		}
		try
		{
			gp_Dir xHint = n.X() * n.X() < 0.9 ? gp_Dir(1, 0, 0) : gp_Dir(0, 1, 0);
			gp_Vec xAxis = gp_Vec(xHint).Crossed(gp_Vec(n));
			xAxis.Normalize();
			gp_Vec yAxis = gp_Vec(n).Crossed(xAxis);
			const double ang = s.bz;
			gp_Vec maj = xAxis * std::cos(ang) + yAxis * std::sin(ang);
			maj.Normalize();
			const gp_Ax2 ax(gp_Pnt(s.ax, s.ay, s.az), n, gp_Dir(maj));
			BRepBuilderAPI_MakeEdge mkEdge(gp_Elips(ax, majorR, minorR));
			if (!mkEdge.IsDone())
				return false;
			BRepBuilderAPI_MakeWire mkWire(mkEdge.Edge());
			if (!mkWire.IsDone())
				return false;
			outWire = mkWire.Wire();
			return !outWire.IsNull();
		}
		catch (...)
		{
			if (errMsg)
				*errMsg = "ellipse edge failed";
			return false;
		}
	}

	BRepBuilderAPI_MakeWire wireMaker;
	int added = 0;
	for (const auto& s : segs)
	{
		TopoDS_Edge edge;
		try
		{
			if (s.kind == SketchCurveSegKind::Arc)
			{
				GC_MakeArcOfCircle mk(gp_Pnt(s.ax, s.ay, s.az), gp_Pnt(s.mx, s.my, s.mz), gp_Pnt(s.bx, s.by, s.bz));
				if (!mk.IsDone())
				{
					if (errMsg)
						*errMsg = "GC_MakeArcOfCircle failed";
					return false;
				}
				BRepBuilderAPI_MakeEdge mkEdge(mk.Value());
				if (!mkEdge.IsDone())
					return false;
				edge = mkEdge.Edge();
			}
			else if (s.kind == SketchCurveSegKind::Line || s.kind == SketchCurveSegKind::SplineThrough)
			{
				BRepBuilderAPI_MakeEdge mkEdge(gp_Pnt(s.ax, s.ay, s.az), gp_Pnt(s.bx, s.by, s.bz));
				if (!mkEdge.IsDone())
					return false;
				edge = mkEdge.Edge();
			}
			else
			{
				if (errMsg)
					*errMsg = "unsupported segment in closed chain";
				return false;
			}
		}
		catch (...)
		{
			if (errMsg)
				*errMsg = "segment edge exception";
			return false;
		}
		wireMaker.Add(edge);
		if (!wireMaker.IsDone())
		{
			if (errMsg)
				*errMsg = "MakeWire Add failed";
			return false;
		}
		++added;
	}
	if (added < 1)
	{
		if (errMsg)
			*errMsg = "no edges";
		return false;
	}
	outWire = wireMaker.Wire();
	return !outWire.IsNull();
}

bool makeClosedFaceFromSegments(const std::vector<SketchCurveSegment>& segs, double planeNx, double planeNy,
								double planeNz, TopoDS_Face& outFace, std::string* errMsg)
{
	TopoDS_Wire wire;
	if (!makeClosedWireFromSegments(segs, planeNx, planeNy, planeNz, wire, errMsg))
		return false;
	BRepBuilderAPI_MakeFace mkFace(wire, Standard_True);
	if (!mkFace.IsDone())
	{
		if (errMsg)
			*errMsg = "MakeFace from segments failed";
		return false;
	}
	outFace = mkFace.Face();
	return true;
}

bool makeFaceFromProfileAndHolePolylinesMm(const std::vector<float>& outerXyzMm,
										  const std::vector<std::vector<float>>& holePolylinesXyzMm, double planeNx,
										  double planeNy, double planeNz, TopoDS_Face& outFace, std::string* errMsg)
{
	TopoDS_Wire outer;
	if (!makeClosedWireFromPolylineMm(outerXyzMm, planeNx, planeNy, planeNz, outer, errMsg))
		return false;
	BRepBuilderAPI_MakeFace mkFace(outer, Standard_True);
	if (!mkFace.IsDone())
	{
		if (errMsg)
			*errMsg = "MakeFace from outer wire failed";
		return false;
	}
	for (const auto& hole : holePolylinesXyzMm)
	{
		if (hole.size() < 9U)
			continue;
		TopoDS_Wire hw;
		if (!makeClosedWireFromPolylineMm(hole, planeNx, planeNy, planeNz, hw, nullptr))
			continue;
		mkFace.Add(hw);
	}
	if (!mkFace.IsDone())
	{
		if (errMsg)
			*errMsg = "MakeFace with holes failed";
		return false;
	}
	outFace = mkFace.Face();
	return true;
}

} // namespace geoalgo
