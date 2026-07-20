/// @file MeshTrajectory.cpp
/// @brief MeshTrajectory 实现

#include "MeshTrajectory.h"

#include "MeshSurfaceReconstruction/NurbsSurfaceFitting.h"
#include "detail/FeatureDiscretizeFrame.h"
#include "detail/OccIncludes.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <map>
#include <sstream>
#include <unordered_map>
#include <utility>
#include <vector>

#include <Geom_BSplineSurface.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <json.hpp>

namespace geoalgo
{
namespace
{
constexpr double kEps = 1e-6;
constexpr double kSnapTolMm = 0.05;

struct Vec3
{
	double x = 0.0;
	double y = 0.0;
	double z = 0.0;
	Vec3() = default;
	Vec3(double ax, double ay, double az) : x(ax), y(ay), z(az) {}
	Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
	Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
	Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
	double dot(const Vec3& o) const { return x * o.x + y * o.y + z * o.z; }
	double length() const { return std::sqrt(dot(*this)); }
	Vec3 normalized() const
	{
		const double l = length();
		return (l > 1e-12) ? (*this * (1.0 / l)) : Vec3{0, 0, 1};
	}
};

Vec3 crossv(const Vec3& a, const Vec3& b)
{
	return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

Vec3 triVertex(const std::vector<float>& soup, int triIndex, int corner)
{
	const std::size_t b = static_cast<std::size_t>(triIndex) * 9U + static_cast<std::size_t>(corner) * 3U;
	return {soup[b], soup[b + 1U], soup[b + 2U]};
}

int triangleCount(const std::vector<float>& soup)
{
	return static_cast<int>(soup.size() / 9U);
}

Vec3 normalizeOrDefault(const double n[3])
{
	Vec3 v{n[0], n[1], n[2]};
	return v.normalized();
}

std::int64_t snapKey(const Vec3& p)
{
	const auto q = [](double v) -> std::int64_t { return static_cast<std::int64_t>(std::llround(v / kSnapTolMm)); };
	return (q(p.x) << 42) ^ (q(p.y) << 21) ^ q(p.z);
}

Vec3 normalizePlaneNormal(const double n[3])
{
	return normalizeOrDefault(n);
}

double signedPlaneDistance(const Vec3& p, const Vec3& origin, const Vec3& n)
{
	return (p - origin).dot(n);
}

bool segmentPlaneIntersect(const Vec3& a, const Vec3& b, const Vec3& origin, const Vec3& n, Vec3& outHit)
{
	const double da = signedPlaneDistance(a, origin, n);
	const double db = signedPlaneDistance(b, origin, n);
	if (std::abs(da) < kEps && std::abs(db) < kEps)
	{
		return false;
	}
	if (std::abs(da) < kEps)
	{
		outHit = a;
		return true;
	}
	if (std::abs(db) < kEps)
	{
		outHit = b;
		return true;
	}
	if (da * db > 0.0)
	{
		return false;
	}
	const double t = da / (da - db);
	outHit = a + (b - a) * t;
	return true;
}

RawPathPoint makePathPoint(const Vec3& p, const Vec3& tangent, const Vec3& normal, bool hasTan, bool hasNrm)
{
	RawPathPoint rp;
	rp.positionMm = {p.x, p.y, p.z};
	if (hasTan)
	{
		rp.tangent = {tangent.x, tangent.y, tangent.z};
		rp.hasTangent = true;
	}
	if (hasNrm)
	{
		rp.normal = {normal.x, normal.y, normal.z};
		rp.hasNormal = true;
	}
	return rp;
}

bool resamplePolylinePoints(std::vector<RawPathPoint>& points, bool closed, double stepMm, bool outputTangent,
							bool outputNormal, const Vec3& defaultNormal)
{
	if (points.size() < 2U || stepMm <= 0.0)
	{
		return true;
	}
	std::vector<double> segLen;
	double total = 0.0;
	for (std::size_t i = 1; i < points.size(); ++i)
	{
		const auto& a = points[i - 1U].positionMm;
		const auto& b = points[i].positionMm;
		const double len = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
		segLen.push_back(len);
		total += len;
	}
	if (closed && points.size() > 1U)
	{
		const auto& a = points.back().positionMm;
		const auto& b = points.front().positionMm;
		const double len = std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
		segLen.push_back(len);
		total += len;
	}
	if (total < stepMm)
	{
		return true;
	}
	const int sampleCount = std::max(2, static_cast<int>(std::ceil(total / stepMm)) + 1);
	std::vector<RawPathPoint> resampled;
	resampled.reserve(static_cast<std::size_t>(sampleCount));
	for (int s = 0; s < sampleCount; ++s)
	{
		const double t = static_cast<double>(s) / static_cast<double>(sampleCount - 1) * total;
		double acc = 0.0;
		std::size_t seg = 0;
		while (seg < segLen.size() && acc + segLen[seg] < t - 1e-9)
		{
			acc += segLen[seg];
			++seg;
		}
		const double local = (seg < segLen.size() && segLen[seg] > 1e-12) ? (t - acc) / segLen[seg] : 0.0;
		const std::size_t i0 = seg % points.size();
		const std::size_t i1 = (i0 + 1U) % points.size();
		const auto& p0 = points[i0].positionMm;
		const auto& p1 = points[i1].positionMm;
		Vec3 pos{p0.x + (p1.x - p0.x) * local, p0.y + (p1.y - p0.y) * local, p0.z + (p1.z - p0.z) * local};
		Vec3 tan{0, 0, 0};
		if (outputTangent)
		{
			tan = Vec3{p1.x - p0.x, p1.y - p0.y, p1.z - p0.z}.normalized();
		}
		resampled.push_back(makePathPoint(pos, tan, defaultNormal, outputTangent, outputNormal));
	}
	points = std::move(resampled);
	return true;
}

std::vector<MeshTrajectoryPolyline> chainSegmentsToPolylines(const std::vector<std::pair<Vec3, Vec3>>& segments)
{
	std::unordered_map<std::int64_t, std::vector<std::pair<Vec3, int>>> adj;
	std::vector<std::pair<Vec3, Vec3>> segStore = segments;
	for (int si = 0; si < static_cast<int>(segStore.size()); ++si)
	{
		const auto& seg = segStore[static_cast<std::size_t>(si)];
		adj[snapKey(seg.first)].push_back({seg.second, si});
		adj[snapKey(seg.second)].push_back({seg.first, si});
	}
	std::vector<bool> used(segStore.size(), false);
	std::vector<MeshTrajectoryPolyline> polylines;

	auto walkChain = [&](const Vec3& start, const Vec3& next, int firstSeg)
	{
		MeshTrajectoryPolyline pl;
		pl.points.push_back(makePathPoint(start, {}, {}, false, false));
		pl.points.push_back(makePathPoint(next, {}, {}, false, false));
		used[static_cast<std::size_t>(firstSeg)] = true;
		Vec3 cur = next;
		for (;;)
		{
			bool extended = false;
			const auto it = adj.find(snapKey(cur));
			if (it == adj.end())
			{
				break;
			}
			for (const auto& nb : it->second)
			{
				if (used[static_cast<std::size_t>(nb.second)])
				{
					continue;
				}
				used[static_cast<std::size_t>(nb.second)] = true;
				pl.points.push_back(makePathPoint(nb.first, {}, {}, false, false));
				cur = nb.first;
				extended = true;
				break;
			}
			if (!extended)
			{
				break;
			}
		}
		if (pl.points.size() >= 2U)
		{
			polylines.push_back(std::move(pl));
		}
	};

	for (int si = 0; si < static_cast<int>(segStore.size()); ++si)
	{
		if (used[static_cast<std::size_t>(si)])
		{
			continue;
		}
		walkChain(segStore[static_cast<std::size_t>(si)].first, segStore[static_cast<std::size_t>(si)].second, si);
	}
	return polylines;
}

double meshTrajectoryPolylineLength(const MeshTrajectoryPolyline& polyline)
{
	double len = 0.0;
	for (std::size_t i = 1; i < polyline.points.size(); ++i)
	{
		const auto& a = polyline.points[i - 1U].positionMm;
		const auto& b = polyline.points[i].positionMm;
		len += std::sqrt((b.x - a.x) * (b.x - a.x) + (b.y - a.y) * (b.y - a.y) + (b.z - a.z) * (b.z - a.z));
	}
	return len;
}

bool discretizeAllMeshTrajectoryPolylines(std::vector<MeshTrajectoryPolyline>& polylines, const double stepMm,
										  const bool outputTangent, const bool outputNormal,
										  const double planeNormalUnit[3], RawPath& outPath, std::string* errMsg)
{
	outPath.points.clear();
	outPath.segmentEndExclusive.clear();
	outPath.closed = false;
	if (polylines.empty())
	{
		if (errMsg)
		{
			*errMsg = "无交线折线";
		}
		return false;
	}
	std::sort(polylines.begin(), polylines.end(),
			  [](const MeshTrajectoryPolyline& a, const MeshTrajectoryPolyline& b)
			  { return meshTrajectoryPolylineLength(a) > meshTrajectoryPolylineLength(b); });
	for (MeshTrajectoryPolyline& pl : polylines)
	{
		if (pl.points.size() < 2U)
		{
			continue;
		}
		(void)discretizeMeshTrajectoryPolyline(pl, stepMm, outputTangent, outputNormal, planeNormalUnit);
		if (pl.points.size() < 2U)
		{
			continue;
		}
		outPath.points.insert(outPath.points.end(), pl.points.begin(), pl.points.end());
		outPath.segmentEndExclusive.push_back(outPath.points.size());
	}
	if (outPath.points.size() < 2U)
	{
		if (errMsg)
		{
			*errMsg = "交线折线点数不足";
		}
		return false;
	}
	return true;
}

bool buildRegionFrame(const std::vector<float>& soup, const std::vector<int>& triIndices, Vec3& outOrigin,
					  Vec3& outNormal, Vec3& outAxisU, Vec3& outAxisV)
{
	if (triIndices.empty())
	{
		return false;
	}
	Vec3 centroid{0, 0, 0};
	Vec3 normalSum{0, 0, 0};
	int count = 0;
	for (const int ti : triIndices)
	{
		const Vec3 v0 = triVertex(soup, ti, 0);
		const Vec3 v1 = triVertex(soup, ti, 1);
		const Vec3 v2 = triVertex(soup, ti, 2);
		centroid = centroid + (v0 + v1 + v2) * (1.0 / 3.0);
		normalSum = normalSum + crossv(v1 - v0, v2 - v0).normalized();
		++count;
	}
	if (count == 0)
	{
		return false;
	}
	outOrigin = centroid * (1.0 / static_cast<double>(count));
	outNormal = normalSum.normalized();
	Vec3 ref = (std::abs(outNormal.z) < 0.9) ? Vec3{0, 0, 1} : Vec3{0, 1, 0};
	outAxisU = crossv(ref, outNormal).normalized();
	outAxisV = crossv(outNormal, outAxisU).normalized();
	return true;
}

int resolveBsplineEdgeCount(const int gridEdge, const double spanMm, const double fitUvSpacingMm)
{
	int fitN = gridEdge;
	if (fitUvSpacingMm > 1e-6 && spanMm > 1e-6)
	{
		fitN = static_cast<int>(std::ceil(spanMm / fitUvSpacingMm));
		fitN = std::max(4, std::min(gridEdge, fitN));
	}
	return std::max(4, fitN);
}

bool fitMeshTrajectoryNurbsFromGrid(const TColgp_Array2OfPnt& grid, const MeshTrajectoryBsplineParams& bspline,
									Handle(Geom_BSplineSurface) & outSurface)
{
	outSurface.Nullify();
	const int nu = grid.UpperRow() - grid.LowerRow() + 1;
	const int nv = grid.UpperCol() - grid.LowerCol() + 1;
	const int degU = std::max(1, bspline.nurbsDegreeU);
	const int degV = std::max(1, bspline.nurbsDegreeV);
	const int ctrlU = meshrecon::resolveControlPointCountFromFitGrid(nu, degU, bspline.controlPointDensityFactor);
	const int ctrlV = meshrecon::resolveControlPointCountFromFitGrid(nv, degV, bspline.controlPointDensityFactor);

	const meshrecon::NurbsFitMode primary = meshrecon::nurbsFitModeFromMeshSurface(bspline.fitMode);
	const meshrecon::NurbsFitMode candidates[] = {
		primary,
		meshrecon::NurbsFitMode::ApproxCentripetalFixedCtrlpts,
		meshrecon::NurbsFitMode::ApproxFixedCtrlpts,
		meshrecon::NurbsFitMode::ApproxCentripetal,
		meshrecon::NurbsFitMode::Interpolate,
	};
	std::vector<meshrecon::NurbsFitMode> uniqueModes;
	uniqueModes.reserve(5);
	for (const meshrecon::NurbsFitMode mode : candidates)
	{
		if (std::find(uniqueModes.begin(), uniqueModes.end(), mode) != uniqueModes.end())
		{
			continue;
		}
		uniqueModes.push_back(mode);
		Handle(Geom_BSplineSurface) surface;
		if (meshrecon::fitNurbsSurfaceFromGrid(grid, ctrlU, ctrlV, mode, degU, degV, surface) && !surface.IsNull())
		{
			outSurface = surface;
			return true;
		}
	}
	return false;
}

bool closestPointOnTriangle(const Vec3& p, const Vec3& a, const Vec3& b, const Vec3& c, Vec3& outClosest)
{
	const Vec3 ab = b - a;
	const Vec3 ac = c - a;
	const Vec3 ap = p - a;
	const double d1 = ab.dot(ap);
	const double d2 = ac.dot(ap);
	if (d1 <= 0.0 && d2 <= 0.0)
	{
		outClosest = a;
		return true;
	}
	const Vec3 bp = p - b;
	const double d3 = ab.dot(bp);
	const double d4 = ac.dot(bp);
	if (d3 >= 0.0 && d4 <= d3)
	{
		outClosest = b;
		return true;
	}
	const double vc = d1 * d4 - d3 * d2;
	if (vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0)
	{
		const double v = d1 / (d1 - d3);
		outClosest = a + ab * v;
		return true;
	}
	const Vec3 cp = p - c;
	const double d5 = ab.dot(cp);
	const double d6 = ac.dot(cp);
	if (d6 >= 0.0 && d5 <= d6)
	{
		outClosest = c;
		return true;
	}
	const double vb = d5 * d2 - d1 * d6;
	if (vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0)
	{
		const double w = d2 / (d2 - d6);
		outClosest = a + ac * w;
		return true;
	}
	const double va = d3 * d6 - d5 * d4;
	if (va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0)
	{
		const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
		outClosest = b + (c - b) * w;
		return true;
	}
	const double denom = 1.0 / (va + vb + vc);
	const double v = vb * denom;
	const double w = vc * denom;
	outClosest = a + ab * v + ac * w;
	return true;
}

Vec3 projectGridPointToRegion(const std::vector<float>& soup, const std::vector<int>& triIndices, const Vec3& origin,
							  const Vec3& rotU, const Vec3& rotV, const double tu, const double tv)
{
	const Vec3 query = origin + rotU * tu + rotV * tv;
	Vec3 best = query;
	double bestDist2 = std::numeric_limits<double>::max();
	for (const int ti : triIndices)
	{
		const Vec3 a = triVertex(soup, ti, 0);
		const Vec3 b = triVertex(soup, ti, 1);
		const Vec3 c = triVertex(soup, ti, 2);
		Vec3 closest;
		(void)closestPointOnTriangle(query, a, b, c, closest);
		const Vec3 d = closest - query;
		const double dist2 = d.dot(d);
		if (dist2 < bestDist2)
		{
			bestDist2 = dist2;
			best = closest;
		}
	}
	return best;
}

RawPathPoint sampleSurfaceAtUv(const Handle(Geom_BSplineSurface) & surface, const double u, const double v,
							   const bool outputTangent, const bool outputNormal)
{
	gp_Pnt p;
	gp_Vec duVec, dvVec;
	surface->D1(u, v, p, duVec, dvVec);
	const Vec3 pos{p.X(), p.Y(), p.Z()};
	const Vec3 du{duVec.X(), duVec.Y(), duVec.Z()};
	const Vec3 dv{dvVec.X(), dvVec.Y(), dvVec.Z()};
	Vec3 tan = du.normalized();
	Vec3 nrm = crossv(du, dv).normalized();
	return makePathPoint(pos, tan, nrm, outputTangent, outputNormal);
}

void appendTraceModePoints(std::vector<RawPathPoint>& grid, const int nu, const int nv,
						   const MeshTrajectoryUvTraceMode traceMode)
{
	if (grid.size() != static_cast<std::size_t>(nu) * static_cast<std::size_t>(nv))
	{
		return;
	}
	std::vector<RawPathPoint> ordered;
	ordered.reserve(grid.size());
	switch (traceMode)
	{
	case MeshTrajectoryUvTraceMode::USerpentine:
		for (int iv = 0; iv < nv; ++iv)
		{
			if (iv % 2 == 0)
			{
				for (int iu = 0; iu < nu; ++iu)
				{
					ordered.push_back(grid[static_cast<std::size_t>(iv * nu + iu)]);
				}
			}
			else
			{
				for (int iu = nu - 1; iu >= 0; --iu)
				{
					ordered.push_back(grid[static_cast<std::size_t>(iv * nu + iu)]);
				}
			}
		}
		break;
	case MeshTrajectoryUvTraceMode::VSerpentine:
		for (int iu = 0; iu < nu; ++iu)
		{
			if (iu % 2 == 0)
			{
				for (int iv = 0; iv < nv; ++iv)
				{
					ordered.push_back(grid[static_cast<std::size_t>(iv * nu + iu)]);
				}
			}
			else
			{
				for (int iv = nv - 1; iv >= 0; --iv)
				{
					ordered.push_back(grid[static_cast<std::size_t>(iv * nu + iu)]);
				}
			}
		}
		break;
	case MeshTrajectoryUvTraceMode::UvGrid:
	default:
		ordered = std::move(grid);
		break;
	}
	grid = std::move(ordered);
}

bool fitBsplineRegionSurface(const std::vector<float>& soup, const std::vector<int>& triangleIndices,
							 const MeshTrajectoryBsplineParams& bspline, Handle(Geom_BSplineSurface) & outSurface,
							 int& outNu, int& outNv, std::string* errMsg)
{
	if (triangleIndices.size() < 3U)
	{
		if (errMsg)
		{
			*errMsg = "B样条区域至少需要 3 个三角面";
		}
		return false;
	}
	Vec3 origin, normal, axisU, axisV;
	if (!buildRegionFrame(soup, triangleIndices, origin, normal, axisU, axisV))
	{
		if (errMsg)
		{
			*errMsg = "无法构建区域坐标系";
		}
		return false;
	}
	const double angleRad = bspline.gridAngleDeg * 3.14159265358979323846 / 180.0;
	const double ca = std::cos(angleRad);
	const double sa = std::sin(angleRad);
	const Vec3 rotU = (axisU * ca + axisV * sa).normalized();
	const Vec3 rotV = crossv(normal, rotU).normalized();

	std::vector<Vec3> samples;
	samples.reserve(triangleIndices.size() * 3U);
	for (const int ti : triangleIndices)
	{
		for (int c = 0; c < 3; ++c)
		{
			samples.push_back(triVertex(soup, ti, c));
		}
	}
	double minU = std::numeric_limits<double>::max();
	double maxU = -std::numeric_limits<double>::max();
	double minV = std::numeric_limits<double>::max();
	double maxV = -std::numeric_limits<double>::max();
	for (const Vec3& p : samples)
	{
		const Vec3 d = p - origin;
		const double u = d.dot(rotU);
		const double v = d.dot(rotV);
		minU = std::min(minU, u);
		maxU = std::max(maxU, u);
		minV = std::min(minV, v);
		maxV = std::max(maxV, v);
	}
	const double uSpan = maxU - minU;
	const double vSpan = maxV - minV;
	int nu = std::max(4, bspline.uvCountU);
	int nv = std::max(4, bspline.uvCountV);
	nu = resolveBsplineEdgeCount(nu, uSpan, bspline.fitUvSpacingMm);
	nv = resolveBsplineEdgeCount(nv, vSpan, bspline.fitUvSpacingMm);
	const double du = uSpan / static_cast<double>(nu - 1);
	const double dv = vSpan / static_cast<double>(nv - 1);
	if (du < 1e-9 || dv < 1e-9)
	{
		if (errMsg)
		{
			*errMsg = "选中区域 UV 跨度太小";
		}
		return false;
	}

	TColgp_Array2OfPnt grid(1, nu, 1, nv);
	for (int iu = 1; iu <= nu; ++iu)
	{
		for (int iv = 1; iv <= nv; ++iv)
		{
			const double tu = minU + du * static_cast<double>(iu - 1);
			const double tv = minV + dv * static_cast<double>(iv - 1);
			const Vec3 bestP = projectGridPointToRegion(soup, triangleIndices, origin, rotU, rotV, tu, tv);
			grid.SetValue(iu, iv, gp_Pnt(bestP.x, bestP.y, bestP.z));
		}
	}

	if (!fitMeshTrajectoryNurbsFromGrid(grid, bspline, outSurface))
	{
		if (errMsg)
		{
			*errMsg = "B样条曲面拟合失败";
		}
		return false;
	}
	outNu = nu;
	outNv = nv;
	return true;
}

void tessellateBsplineSurface(const Handle(Geom_BSplineSurface) & surface, const int divU, const int divV,
							  std::vector<float>& outTriangleSoupModel)
{
	outTriangleSoupModel.clear();
	if (surface.IsNull() || divU < 2 || divV < 2)
	{
		return;
	}
	double uMin = 0.0;
	double uMax = 1.0;
	double vMin = 0.0;
	double vMax = 1.0;
	surface->Bounds(uMin, uMax, vMin, vMax);
	const int nu = divU;
	const int nv = divV;
	std::vector<Vec3> grid;
	grid.reserve(static_cast<std::size_t>(nu) * static_cast<std::size_t>(nv));
	for (int iv = 0; iv < nv; ++iv)
	{
		for (int iu = 0; iu < nu; ++iu)
		{
			const double u = uMin + (uMax - uMin) * static_cast<double>(iu) / static_cast<double>(nu - 1);
			const double v = vMin + (vMax - vMin) * static_cast<double>(iv) / static_cast<double>(nv - 1);
			gp_Pnt p;
			surface->D0(u, v, p);
			grid.push_back({p.X(), p.Y(), p.Z()});
		}
	}
	outTriangleSoupModel.reserve(static_cast<std::size_t>((nu - 1) * (nv - 1) * 9U));
	for (int iv = 0; iv < nv - 1; ++iv)
	{
		for (int iu = 0; iu < nu - 1; ++iu)
		{
			const Vec3& a = grid[static_cast<std::size_t>(iv * nu + iu)];
			const Vec3& b = grid[static_cast<std::size_t>(iv * nu + iu + 1)];
			const Vec3& c = grid[static_cast<std::size_t>((iv + 1) * nu + iu)];
			const Vec3& d = grid[static_cast<std::size_t>((iv + 1) * nu + iu + 1)];
			const Vec3 tris[2][3] = {{a, b, c}, {b, d, c}};
			for (const auto& tri : tris)
			{
				for (const Vec3& p : tri)
				{
					outTriangleSoupModel.push_back(static_cast<float>(p.x));
					outTriangleSoupModel.push_back(static_cast<float>(p.y));
					outTriangleSoupModel.push_back(static_cast<float>(p.z));
				}
			}
		}
	}
}

bool generateBsplineRegionPath(const std::vector<float>& soup, const MeshTrajectorySpec& spec, RawPath& outPath,
							   std::string* errMsg)
{
	Handle(Geom_BSplineSurface) surface;
	int nu = 0;
	int nv = 0;
	if (!fitBsplineRegionSurface(soup, spec.region.triangleIndices, spec.bspline, surface, nu, nv, errMsg))
	{
		return false;
	}
	outPath.points.clear();
	outPath.closed = false;
	const int outU = std::max(2, nu);
	const int outV = std::max(2, nv);
	double uMin = 0.0;
	double uMax = 1.0;
	double vMin = 0.0;
	double vMax = 1.0;
	surface->Bounds(uMin, uMax, vMin, vMax);
	std::vector<RawPathPoint> gridPts;
	gridPts.reserve(static_cast<std::size_t>(outU) * static_cast<std::size_t>(outV));
	for (int iv = 0; iv < outV; ++iv)
	{
		for (int iu = 0; iu < outU; ++iu)
		{
			const double u = uMin + (uMax - uMin) * static_cast<double>(iu) / static_cast<double>(outU - 1);
			const double v = vMin + (vMax - vMin) * static_cast<double>(iv) / static_cast<double>(outV - 1);
			gridPts.push_back(
				sampleSurfaceAtUv(surface, u, v, spec.discretize.outputTangent, spec.discretize.outputNormal));
		}
	}
	appendTraceModePoints(gridPts, outU, outV, spec.bspline.traceMode);
	RawPath framed;
	framed.points = std::move(gridPts);
	detail::assignPathChordTangents(framed, false, spec.discretize.outputTangent);
	outPath.points = std::move(framed.points);
	return true;
}

} // namespace

bool buildBsplineRegionSurfacePreview(const std::vector<float>& triangleSoup, const MeshTrajectoryRegion& region,
									  const MeshTrajectoryBsplineParams& bspline,
									  std::vector<float>& outTriangleSoupModel, std::string* errMsg)
{
	Handle(Geom_BSplineSurface) surface;
	int nu = 0;
	int nv = 0;
	if (!fitBsplineRegionSurface(triangleSoup, region.triangleIndices, bspline, surface, nu, nv, errMsg))
	{
		return false;
	}
	const int divU = std::max(16, std::min(nu, 48));
	const int divV = std::max(16, std::min(nv, 48));
	tessellateBsplineSurface(surface, divU, divV, outTriangleSoupModel);
	return !outTriangleSoupModel.empty();
}

bool validateMeshTrajectorySpec(const MeshTrajectorySpec& spec, std::string* errMsg)
{
	if (spec.workpiece.backendIdUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "workpiece.backendIdUtf8 为空";
		}
		return false;
	}
	if (spec.method == MeshTrajectoryMethod::BsplineRegion && spec.region.triangleIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "B样条法需要选中三角区域";
		}
		return false;
	}
	if (spec.method == MeshTrajectoryMethod::CrossSection)
	{
		const Vec3 n = normalizePlaneNormal(spec.crossSection.planeNormal);
		if (n.length() < 0.5)
		{
			if (errMsg)
			{
				*errMsg = "截面法向无效";
			}
			return false;
		}
	}
	return true;
}

bool filterSoupByTriangleIndices(const std::vector<float>& triangleSoup, const std::vector<int>& triangleIndices,
								 std::vector<float>& outSoup, std::vector<int>& outOriginalTriangleIndices)
{
	outSoup.clear();
	outOriginalTriangleIndices.clear();
	if (triangleIndices.empty())
	{
		return true;
	}
	const int triCount = triangleCount(triangleSoup);
	for (const int ti : triangleIndices)
	{
		if (ti < 0 || ti >= triCount)
		{
			continue;
		}
		outOriginalTriangleIndices.push_back(ti);
		const std::size_t b = static_cast<std::size_t>(ti) * 9U;
		outSoup.insert(outSoup.end(), triangleSoup.begin() + static_cast<std::ptrdiff_t>(b),
					   triangleSoup.begin() + static_cast<std::ptrdiff_t>(b + 9U));
	}
	return !outSoup.empty();
}

bool intersectPlaneWithTriangleSoup(const std::vector<float>& triangleSoup, const double planeOriginMm[3],
									const double planeNormalUnit[3], const std::vector<int>* triangleIndexFilter,
									std::vector<MeshTrajectoryPolyline>& outPolylines, std::string* errMsg)
{
	outPolylines.clear();
	const Vec3 origin{planeOriginMm[0], planeOriginMm[1], planeOriginMm[2]};
	const Vec3 n = normalizePlaneNormal(planeNormalUnit);
	const int triCount = triangleCount(triangleSoup);
	std::vector<std::pair<Vec3, Vec3>> segments;

	auto processTri = [&](int ti)
	{
		const Vec3 v0 = triVertex(triangleSoup, ti, 0);
		const Vec3 v1 = triVertex(triangleSoup, ti, 1);
		const Vec3 v2 = triVertex(triangleSoup, ti, 2);
		const Vec3 verts[3] = {v0, v1, v2};
		Vec3 hits[3];
		int hitCount = 0;
		for (int e = 0; e < 3; ++e)
		{
			Vec3 hit;
			if (segmentPlaneIntersect(verts[e], verts[(e + 1) % 3], origin, n, hit))
			{
				bool dup = false;
				for (int h = 0; h < hitCount; ++h)
				{
					if ((hits[h] - hit).length() < kSnapTolMm)
					{
						dup = true;
						break;
					}
				}
				if (!dup)
				{
					hits[hitCount++] = hit;
				}
			}
		}
		if (hitCount == 2)
		{
			segments.push_back({hits[0], hits[1]});
		}
	};

	if (triangleIndexFilter && !triangleIndexFilter->empty())
	{
		for (const int ti : *triangleIndexFilter)
		{
			if (ti >= 0 && ti < triCount)
			{
				processTri(ti);
			}
		}
	}
	else
	{
		for (int ti = 0; ti < triCount; ++ti)
		{
			processTri(ti);
		}
	}

	if (segments.empty())
	{
		if (errMsg)
		{
			*errMsg = "平面与网格无交线";
		}
		return false;
	}
	outPolylines = chainSegmentsToPolylines(segments);
	return !outPolylines.empty();
}

bool discretizeMeshTrajectoryPolyline(MeshTrajectoryPolyline& polyline, double stepMm, bool outputTangent,
									  bool outputNormal, const double planeNormalUnit[3])
{
	const Vec3 n = normalizeOrDefault(planeNormalUnit) * -1.0;
	if (polyline.points.size() >= 2U && outputTangent)
	{
		for (std::size_t i = 0; i < polyline.points.size(); ++i)
		{
			const std::size_t j = std::min(i + 1U, polyline.points.size() - 1U);
			const auto& a = polyline.points[i].positionMm;
			const auto& b = polyline.points[j].positionMm;
			Vec3 tan{b.x - a.x, b.y - a.y, b.z - a.z};
			polyline.points[i] = makePathPoint({a.x, a.y, a.z}, tan.normalized(), n, true, outputNormal);
		}
	}
	return resamplePolylinePoints(polyline.points, polyline.closed, stepMm, outputTangent, outputNormal, n);
}

bool generateMeshTrajectory(const MeshTrajectorySpec& spec, const std::vector<float>& triangleSoup, RawPath& outPath,
							std::string* errMsg)
{
	if (!validateMeshTrajectorySpec(spec, errMsg))
	{
		return false;
	}
	if (triangleSoup.size() < 9U)
	{
		if (errMsg)
		{
			*errMsg = "三角网格为空";
		}
		return false;
	}
	outPath.points.clear();
	outPath.closed = false;

	if (spec.method == MeshTrajectoryMethod::BsplineRegion)
	{
		return generateBsplineRegionPath(triangleSoup, spec, outPath, errMsg);
	}

	std::vector<MeshTrajectoryPolyline> polylines;
	const std::vector<int>* filter = spec.region.triangleIndices.empty() ? nullptr : &spec.region.triangleIndices;
	if (!intersectPlaneWithTriangleSoup(triangleSoup, spec.crossSection.planeOriginMm, spec.crossSection.planeNormal,
										filter, polylines, errMsg))
	{
		return false;
	}
	return discretizeAllMeshTrajectoryPolylines(polylines, spec.discretize.stepMm, spec.discretize.outputTangent,
												spec.discretize.outputNormal, spec.crossSection.planeNormal, outPath,
												errMsg);
}

bool meshTrajectorySpecFromJson(const std::string& jsonUtf8, MeshTrajectorySpec& out, std::string* errMsg)
{
	try
	{
		const auto j = nlohmann::json::parse(jsonUtf8);
		out = MeshTrajectorySpec{};
		out.schemaVersion = j.value("schemaVersion", 1);
		out.trajectoryId = j.value("trajectoryId", std::string{});
		if (j.contains("workpiece"))
		{
			out.workpiece.backendIdUtf8 = j["workpiece"].value("backendIdUtf8", std::string{});
			out.workpiece.frameId = j["workpiece"].value("frameId", std::string{"workpiece"});
		}
		const std::string method = j.value("method", std::string{"CrossSection"});
		out.method =
			(method == "BsplineRegion") ? MeshTrajectoryMethod::BsplineRegion : MeshTrajectoryMethod::CrossSection;
		if (j.contains("region") && j["region"].contains("triangleIndices"))
		{
			out.region.triangleIndices = j["region"]["triangleIndices"].get<std::vector<int>>();
		}
		if (j.contains("crossSection"))
		{
			const auto& cs = j["crossSection"];
			if (cs.contains("planeOriginMm") && cs["planeOriginMm"].is_array() && cs["planeOriginMm"].size() >= 3U)
			{
				for (int i = 0; i < 3; ++i)
				{
					out.crossSection.planeOriginMm[i] = cs["planeOriginMm"][i].get<double>();
				}
			}
			if (cs.contains("planeNormal") && cs["planeNormal"].is_array() && cs["planeNormal"].size() >= 3U)
			{
				for (int i = 0; i < 3; ++i)
				{
					out.crossSection.planeNormal[i] = cs["planeNormal"][i].get<double>();
				}
			}
		}
		if (j.contains("discretize"))
		{
			const auto& d = j["discretize"];
			out.discretize.stepMm = d.value("stepMm", 2.0);
			out.discretize.outputTangent = d.value("outputTangent", true);
			out.discretize.outputNormal = d.value("outputNormal", true);
		}
		if (j.contains("bspline"))
		{
			const auto& b = j["bspline"];
			out.bspline.uvCountU = b.value("uvCountU", 16);
			out.bspline.uvCountV = b.value("uvCountV", 16);
			out.bspline.gridAngleDeg = b.value("gridAngleDeg", 0.0);
			out.bspline.fitUvSpacingMm = b.value("fitUvSpacingMm", 0.0);
			const std::string trace = b.value("traceMode", std::string("USerpentine"));
			if (trace == "VSerpentine")
			{
				out.bspline.traceMode = MeshTrajectoryUvTraceMode::VSerpentine;
			}
			else if (trace == "UvGrid")
			{
				out.bspline.traceMode = MeshTrajectoryUvTraceMode::UvGrid;
			}
			else
			{
				out.bspline.traceMode = MeshTrajectoryUvTraceMode::USerpentine;
			}
			if (b.contains("fitMode"))
			{
				const int fitModeInt = b["fitMode"].is_number_integer() ? b["fitMode"].get<int>() : 4;
				switch (fitModeInt)
				{
				case 1:
					out.bspline.fitMode = MeshSurfaceNurbsFitMode::Interpolate;
					break;
				case 2:
					out.bspline.fitMode = MeshSurfaceNurbsFitMode::ApproxFixedCtrlpts;
					break;
				case 3:
					out.bspline.fitMode = MeshSurfaceNurbsFitMode::ApproxCentripetal;
					break;
				case 4:
				default:
					out.bspline.fitMode = MeshSurfaceNurbsFitMode::ApproxCentripetalFixedCtrlpts;
					break;
				}
			}
			out.bspline.controlPointDensityFactor = b.value("controlPointDensityFactor", 0.5);
			out.bspline.nurbsDegreeU = b.value("nurbsDegreeU", 3);
			out.bspline.nurbsDegreeV = b.value("nurbsDegreeV", 3);
		}
		return true;
	}
	catch (const std::exception& ex)
	{
		if (errMsg)
		{
			*errMsg = ex.what();
		}
		return false;
	}
}

bool meshTrajectorySpecToJson(const MeshTrajectorySpec& spec, std::string& outJsonUtf8)
{
	nlohmann::json j;
	j["schemaVersion"] = spec.schemaVersion;
	j["trajectoryId"] = spec.trajectoryId;
	j["workpiece"]["backendIdUtf8"] = spec.workpiece.backendIdUtf8;
	j["workpiece"]["frameId"] = spec.workpiece.frameId;
	j["method"] = (spec.method == MeshTrajectoryMethod::BsplineRegion) ? "BsplineRegion" : "CrossSection";
	j["region"]["triangleIndices"] = spec.region.triangleIndices;
	j["crossSection"]["planeOriginMm"] = {spec.crossSection.planeOriginMm[0], spec.crossSection.planeOriginMm[1],
										  spec.crossSection.planeOriginMm[2]};
	j["crossSection"]["planeNormal"] = {spec.crossSection.planeNormal[0], spec.crossSection.planeNormal[1],
										spec.crossSection.planeNormal[2]};
	j["discretize"]["stepMm"] = spec.discretize.stepMm;
	j["discretize"]["outputTangent"] = spec.discretize.outputTangent;
	j["discretize"]["outputNormal"] = spec.discretize.outputNormal;
	j["bspline"]["uvCountU"] = spec.bspline.uvCountU;
	j["bspline"]["uvCountV"] = spec.bspline.uvCountV;
	j["bspline"]["gridAngleDeg"] = spec.bspline.gridAngleDeg;
	j["bspline"]["fitUvSpacingMm"] = spec.bspline.fitUvSpacingMm;
	switch (spec.bspline.traceMode)
	{
	case MeshTrajectoryUvTraceMode::VSerpentine:
		j["bspline"]["traceMode"] = "VSerpentine";
		break;
	case MeshTrajectoryUvTraceMode::UvGrid:
		j["bspline"]["traceMode"] = "UvGrid";
		break;
	default:
		j["bspline"]["traceMode"] = "USerpentine";
		break;
	}
	j["bspline"]["fitMode"] = static_cast<int>(spec.bspline.fitMode);
	j["bspline"]["controlPointDensityFactor"] = spec.bspline.controlPointDensityFactor;
	j["bspline"]["nurbsDegreeU"] = spec.bspline.nurbsDegreeU;
	j["bspline"]["nurbsDegreeV"] = spec.bspline.nurbsDegreeV;
	outJsonUtf8 = j.dump();
	return true;
}

} // namespace geoalgo
