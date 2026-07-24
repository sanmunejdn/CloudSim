/// @file MeshDiscretize.cpp
/// @brief MeshDiscretize 实现

#include "MeshDiscretize.h"

#include "Discretize.h"
#include "ShapeIo.h"
#include "detail/OccIncludes.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <new>
#include <utility>

namespace geoalgo
{
namespace
{
constexpr int kTriangleCountSearchIters = 8;
constexpr double kTriangleCountRelTol = 0.15;
constexpr double kDeflectionSearchLo = 1e-4;
constexpr double kDeflectionSearchHi = 0.2;

bool prepareDensityControl(MeshDiscretizeParams& p, std::string* errMsg)
{
	if (p.densityControl == MeshDensityControl::QualityPreset)
	{
		applyQualityPreset(p);
		return true;
	}

	p.quality = MeshQualityPreset::Custom;
	if (p.densityControl == MeshDensityControl::TargetEdgeLength)
	{
		if (!(p.targetEdgeLengthMm > 0.0))
		{
			if (errMsg)
			{
				*errMsg = "targetEdgeLengthMm must be > 0";
			}
			return false;
		}
		// 略密基网格；角偏差用 1°（勿用 0.5° 把圆角打过密，否则 refine 只增不减）
		p.tessellate.linearDeflectionRelative = false;
		p.tessellate.linearDeflectionMm = std::clamp(p.targetEdgeLengthMm * 0.25, 1e-4, 1.0e3);
		p.tessellate.angularDeflectionDeg = 1.0;
		return true;
	}

	if (p.densityControl == MeshDensityControl::TargetTriangleCount)
	{
		if (p.targetTriangleCount == 0U)
		{
			if (errMsg)
			{
				*errMsg = "targetTriangleCount must be > 0";
			}
			return false;
		}
		return true;
	}

	applyQualityPreset(p);
	return true;
}

void appendTri9(std::vector<float>& out, float ax, float ay, float az, float bx, float by, float bz, float cx, float cy,
				float cz)
{
	out.push_back(ax);
	out.push_back(ay);
	out.push_back(az);
	out.push_back(bx);
	out.push_back(by);
	out.push_back(bz);
	out.push_back(cx);
	out.push_back(cy);
	out.push_back(cz);
}

double triEdgeLen2(float ax, float ay, float az, float bx, float by, float bz)
{
	const double dx = static_cast<double>(bx) - static_cast<double>(ax);
	const double dy = static_cast<double>(by) - static_cast<double>(ay);
	const double dz = static_cast<double>(bz) - static_cast<double>(az);
	return dx * dx + dy * dy + dz * dz;
}

bool discretizeShapeByTriangleCount(const TopoDS_Shape& shape, MeshDiscretizeParams& p, std::vector<float>& soup,
									std::string* errMsg)
{
	const std::size_t target = p.targetTriangleCount;
	double lo = kDeflectionSearchLo;
	double hi = kDeflectionSearchHi;
	std::vector<float> bestSoup;
	std::size_t bestDiff = std::numeric_limits<std::size_t>::max();
	double bestDeflection = p.tessellate.linearDeflectionMm;

	for (int iter = 0; iter < kTriangleCountSearchIters; ++iter)
	{
		const double mid = std::sqrt(lo * hi);
		TessellateParams trialParams = p.tessellate;
		trialParams.linearDeflectionRelative = true;
		trialParams.linearDeflectionMm = mid;
		if (!(trialParams.angularDeflectionDeg > 0.0))
		{
			trialParams.angularDeflectionDeg = 0.5;
		}

		std::vector<float> trial;
		if (!discretizeShapeToSoup(shape, trialParams, trial, errMsg))
		{
			return false;
		}

		const std::size_t count = trial.size() / 9U;
		const std::size_t diff = count > target ? count - target : target - count;
		if (diff < bestDiff)
		{
			bestDiff = diff;
			bestSoup = std::move(trial);
			bestDeflection = mid;
		}

		const double relErr = static_cast<double>(diff) / static_cast<double>(std::max<std::size_t>(1U, target));
		if (relErr <= kTriangleCountRelTol)
		{
			break;
		}

		if (count < target)
		{
			hi = mid;
		}
		else
		{
			lo = mid;
		}
	}

	if (bestSoup.empty())
	{
		if (errMsg)
		{
			*errMsg = "triangle-count search produced empty mesh";
		}
		return false;
	}

	p.tessellate.linearDeflectionRelative = true;
	p.tessellate.linearDeflectionMm = bestDeflection;
	soup = std::move(bestSoup);
	return true;
}

void pushTriSoup(std::vector<float>& soup, const gp_Pnt& a, const gp_Pnt& b, const gp_Pnt& c)
{
	soup.push_back(static_cast<float>(a.X()));
	soup.push_back(static_cast<float>(a.Y()));
	soup.push_back(static_cast<float>(a.Z()));
	soup.push_back(static_cast<float>(b.X()));
	soup.push_back(static_cast<float>(b.Y()));
	soup.push_back(static_cast<float>(b.Z()));
	soup.push_back(static_cast<float>(c.X()));
	soup.push_back(static_cast<float>(c.Y()));
	soup.push_back(static_cast<float>(c.Z()));
}

bool discretizeFaceUVGridInternal(const TopoDS_Face& face, const MeshDiscretizeParams& params, std::vector<float>& soup,
								  std::string* errMsg)
{
	const BRepAdaptor_Surface surf(face);
	const double u0 = surf.FirstUParameter();
	const double u1 = surf.LastUParameter();
	const double v0 = surf.FirstVParameter();
	const double v1 = surf.LastVParameter();
	const int nu = std::max(2, params.uvGridCountU);
	const int nv = std::max(2, params.uvGridCountV);
	const bool reversed = face.Orientation() == TopAbs_REVERSED;

	for (int iv = 0; iv < nv - 1; ++iv)
	{
		for (int iu = 0; iu < nu - 1; ++iu)
		{
			const double fu0 = static_cast<double>(iu) / static_cast<double>(nu - 1);
			const double fu1 = static_cast<double>(iu + 1) / static_cast<double>(nu - 1);
			const double fv0 = static_cast<double>(iv) / static_cast<double>(nv - 1);
			const double fv1 = static_cast<double>(iv + 1) / static_cast<double>(nv - 1);
			const gp_Pnt p00 = surf.Value(u0 + (u1 - u0) * fu0, v0 + (v1 - v0) * fv0);
			const gp_Pnt p10 = surf.Value(u0 + (u1 - u0) * fu1, v0 + (v1 - v0) * fv0);
			const gp_Pnt p01 = surf.Value(u0 + (u1 - u0) * fu0, v0 + (v1 - v0) * fv1);
			const gp_Pnt p11 = surf.Value(u0 + (u1 - u0) * fu1, v0 + (v1 - v0) * fv1);
			if (reversed)
			{
				pushTriSoup(soup, p00, p11, p10);
				pushTriSoup(soup, p00, p01, p11);
			}
			else
			{
				pushTriSoup(soup, p00, p10, p11);
				pushTriSoup(soup, p00, p11, p01);
			}
		}
	}
	if (soup.empty())
	{
		if (errMsg)
		{
			*errMsg = "UV grid produced empty soup";
		}
		return false;
	}
	return true;
}

gp_Vec segmentDir(const Polyline3d& poly, std::size_t segStart)
{
	const float* p0 = &poly.xyz[segStart];
	const float* p1 = &poly.xyz[segStart + 3U];
	gp_Vec v(static_cast<double>(p1[0] - p0[0]), static_cast<double>(p1[1] - p0[1]),
			 static_cast<double>(p1[2] - p0[2]));
	if (v.Magnitude() < 1e-9)
	{
		return gp_Vec(0.0, 0.0, 1.0);
	}
	return v.Normalized();
}

gp_Vec orthogonalFrame(const gp_Vec& tangent)
{
	gp_Vec ref(0.0, 0.0, 1.0);
	if (std::abs(tangent.Dot(ref)) > 0.95)
	{
		ref = gp_Vec(0.0, 1.0, 0.0);
	}
	gp_Vec binormal = tangent.Crossed(ref);
	if (binormal.Magnitude() < 1e-9)
	{
		binormal = gp_Vec(1.0, 0.0, 0.0);
	}
	return binormal.Normalized();
}

bool buildTubeAlongPolyline(const Polyline3d& poly, const MeshDiscretizeParams& params, std::vector<float>& soup,
							std::string* errMsg)
{
	if (poly.xyz.size() < 6U)
	{
		if (errMsg)
		{
			*errMsg = "polyline too short for tube";
		}
		return false;
	}
	const int sides = std::max(3, params.tubeSides);
	const double radius = params.tubeRadiusMm;
	const std::size_t nPts = poly.xyz.size() / 3U;

	auto pointAt = [&](std::size_t i)
	{
		const float* p = &poly.xyz[i * 3U];
		return gp_Pnt(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]));
	};

	std::vector<std::vector<gp_Pnt>> rings;
	rings.resize(nPts);
	for (std::size_t i = 0; i < nPts; ++i)
	{
		gp_Vec tangent;
		if (i + 1U < nPts)
		{
			tangent = gp_Vec(pointAt(i), pointAt(i + 1U));
		}
		else
		{
			tangent = gp_Vec(pointAt(i - 1U), pointAt(i));
		}
		if (tangent.Magnitude() < 1e-9)
		{
			tangent = gp_Vec(0.0, 0.0, 1.0);
		}
		tangent.Normalize();
		const gp_Vec binormal = orthogonalFrame(tangent);
		const gp_Vec normal = tangent.Crossed(binormal).Normalized();
		const gp_Pnt center = pointAt(i);
		rings[i].resize(static_cast<std::size_t>(sides));
		for (int s = 0; s < sides; ++s)
		{
			const double ang = (2.0 * 3.14159265358979323846 * static_cast<double>(s)) / static_cast<double>(sides);
			const gp_Vec offset = binormal * (std::cos(ang) * radius) + normal * (std::sin(ang) * radius);
			rings[i][static_cast<std::size_t>(s)] = center.Translated(offset);
		}
	}

	for (std::size_t i = 0; i + 1U < nPts; ++i)
	{
		for (int s = 0; s < sides; ++s)
		{
			const int sn = (s + 1) % sides;
			const gp_Pnt& a = rings[i][static_cast<std::size_t>(s)];
			const gp_Pnt& b = rings[i][static_cast<std::size_t>(sn)];
			const gp_Pnt& c = rings[i + 1U][static_cast<std::size_t>(sn)];
			const gp_Pnt& d = rings[i + 1U][static_cast<std::size_t>(s)];
			pushTriSoup(soup, a, b, c);
			pushTriSoup(soup, a, c, d);
		}
	}
	return !soup.empty();
}

bool buildRibbonAlongPolyline(const Polyline3d& poly, const MeshDiscretizeParams& params, std::vector<float>& soup,
							  std::string* errMsg)
{
	if (poly.xyz.size() < 6U)
	{
		if (errMsg)
		{
			*errMsg = "polyline too short for ribbon";
		}
		return false;
	}
	const double halfW = params.ribbonWidthMm * 0.5;
	const std::size_t nPts = poly.xyz.size() / 3U;
	auto pointAt = [&](std::size_t i)
	{
		const float* p = &poly.xyz[i * 3U];
		return gp_Pnt(static_cast<double>(p[0]), static_cast<double>(p[1]), static_cast<double>(p[2]));
	};

	std::vector<gp_Pnt> left;
	std::vector<gp_Pnt> right;
	left.reserve(nPts);
	right.reserve(nPts);
	for (std::size_t i = 0; i < nPts; ++i)
	{
		gp_Vec tangent;
		if (i + 1U < nPts)
		{
			tangent = gp_Vec(pointAt(i), pointAt(i + 1U));
		}
		else
		{
			tangent = gp_Vec(pointAt(i - 1U), pointAt(i));
		}
		if (tangent.Magnitude() < 1e-9)
		{
			tangent = gp_Vec(0.0, 0.0, 1.0);
		}
		tangent.Normalize();
		const gp_Vec side = orthogonalFrame(tangent) * halfW;
		const gp_Pnt c = pointAt(i);
		left.push_back(c.Translated(side));
		right.push_back(c.Translated(side.Reversed()));
	}
	for (std::size_t i = 0; i + 1U < nPts; ++i)
	{
		pushTriSoup(soup, left[i], right[i], right[i + 1U]);
		pushTriSoup(soup, left[i], right[i + 1U], left[i + 1U]);
	}
	return !soup.empty();
}

} // namespace

void fillMeshReport(const std::vector<float>& soup, MeshDiscretizeReport& report)
{
	report.triangleCount = soup.size() / 9U;
	if (soup.size() < 9U)
	{
		report.bboxDiagonalMm = 0.0;
		report.avgEdgeLengthMm = 0.0;
		return;
	}
	double xmin = std::numeric_limits<double>::max();
	double ymin = xmin;
	double zmin = xmin;
	double xmax = std::numeric_limits<double>::lowest();
	double ymax = xmax;
	double zmax = xmax;
	double edgeSum = 0.0;
	std::size_t edgeCount = 0U;
	for (std::size_t t = 0; t + 8U < soup.size(); t += 9U)
	{
		for (int v = 0; v < 3; ++v)
		{
			const double x = soup[t + static_cast<std::size_t>(v) * 3U];
			const double y = soup[t + static_cast<std::size_t>(v) * 3U + 1U];
			const double z = soup[t + static_cast<std::size_t>(v) * 3U + 2U];
			xmin = std::min(xmin, x);
			ymin = std::min(ymin, y);
			zmin = std::min(zmin, z);
			xmax = std::max(xmax, x);
			ymax = std::max(ymax, y);
			zmax = std::max(zmax, z);
		}
		for (int e = 0; e < 3; ++e)
		{
			const std::size_t i0 = t + static_cast<std::size_t>(e) * 3U;
			const std::size_t i1 = t + static_cast<std::size_t>((e + 1) % 3) * 3U;
			const double dx = soup[i1] - soup[i0];
			const double dy = soup[i1 + 1U] - soup[i0 + 1U];
			const double dz = soup[i1 + 2U] - soup[i0 + 2U];
			edgeSum += std::sqrt(dx * dx + dy * dy + dz * dz);
			++edgeCount;
		}
	}
	const double dx = xmax - xmin;
	const double dy = ymax - ymin;
	const double dz = zmax - zmin;
	report.bboxDiagonalMm = std::sqrt(dx * dx + dy * dy + dz * dz);
	report.avgEdgeLengthMm = edgeCount > 0U ? edgeSum / static_cast<double>(edgeCount) : 0.0;
}

bool discretizeFaceToMesh(const TopoDS_Face& face, const MeshDiscretizeParams& params, std::vector<float>& soup,
						  std::string* errMsg)
{
	MeshDiscretizeParams p = params;
	applyQualityPreset(p);
	if (p.mode == MeshDiscretizeMode::UVStructuredGrid)
	{
		return discretizeFaceUVGridInternal(face, p, soup, errMsg);
	}
	return discretizeFaceToSoup(face, p.tessellate, soup, errMsg);
}

bool discretizeShapeToMesh(const TopoDS_Shape& shape, const MeshDiscretizeParams& params, std::vector<float>& soup,
						   MeshDiscretizeReport& report, std::string* errMsg)
{
	soup.clear();
	MeshDiscretizeParams p = params;
	if (!prepareDensityControl(p, errMsg))
	{
		return false;
	}
	report.modeUsed = p.mode;

	switch (p.mode)
	{
	case MeshDiscretizeMode::UVStructuredGrid:
		for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
		{
			const TopoDS_Face face = TopoDS::Face(exp.Current());
			if (!discretizeFaceUVGridInternal(face, p, soup, errMsg))
			{
				return false;
			}
		}
		break;
	case MeshDiscretizeMode::WireTubeMesh:
	case MeshDiscretizeMode::WireRibbonMesh:
	{
		Polyline3d poly;
		bool gotPoly = false;
		for (TopExp_Explorer exp(shape, TopAbs_WIRE); exp.More(); exp.Next())
		{
			const TopoDS_Wire wire = TopoDS::Wire(exp.Current());
			if (discretizeWire(wire, p.tessellate, poly, errMsg))
			{
				gotPoly = true;
				break;
			}
		}
		if (!gotPoly)
		{
			for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next())
			{
				const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
				if (discretizeEdge(edge, p.tessellate, poly, errMsg))
				{
					gotPoly = true;
					break;
				}
			}
		}
		if (!gotPoly)
		{
			if (errMsg)
			{
				*errMsg = "no wire/edge for sweep mesh";
			}
			return false;
		}
		if (p.mode == MeshDiscretizeMode::WireTubeMesh)
		{
			if (!buildTubeAlongPolyline(poly, p, soup, errMsg))
			{
				return false;
			}
			break;
		}
		if (!buildRibbonAlongPolyline(poly, p, soup, errMsg))
		{
			return false;
		}
		break;
	}
	case MeshDiscretizeMode::RemeshSoup:
	case MeshDiscretizeMode::PointCloudSurface:
	case MeshDiscretizeMode::ProfileSweepMesh:
		if (errMsg)
		{
			*errMsg = "mesh mode not implemented in this build";
		}
		return false;
	case MeshDiscretizeMode::AdaptiveTriangulation:
	case MeshDiscretizeMode::UniformRelative:
	default:
		if (p.densityControl == MeshDensityControl::TargetTriangleCount)
		{
			if (!discretizeShapeByTriangleCount(shape, p, soup, errMsg))
			{
				return false;
			}
		}
		else if (!discretizeShapeToSoup(shape, p.tessellate, soup, errMsg))
		{
			return false;
		}
		// 平面预加密到约 1.5×目标边长，严格目标交给 Data remesh；过密则 remesh 拉稀
		if (p.densityControl == MeshDensityControl::TargetEdgeLength)
		{
			const double preRefineMaxEdgeMm = p.targetEdgeLengthMm * 1.5;
			if (!refineTriangleSoupToMaxEdge(soup, preRefineMaxEdgeMm, errMsg))
			{
				return false;
			}
		}
		break;
	}
	fillMeshReport(soup, report);
	return !soup.empty();
}

bool refineTriangleSoupToMaxEdge(std::vector<float>& soup, double maxEdgeMm, std::string* errMsg)
{
	if (!(maxEdgeMm > 0.0))
	{
		if (errMsg)
		{
			*errMsg = "maxEdgeMm must be > 0";
		}
		return false;
	}
	if (soup.size() < 9U)
	{
		return true;
	}

	try
	{
		// 给后续 isotropicRemesh 留预算；顶到 200 万会跳过 remesh 面数失控
		constexpr std::size_t kMaxTris = 400000U;
		const double maxE2 = maxEdgeMm * maxEdgeMm;
		constexpr int kMaxPasses = 48;

		for (int pass = 0; pass < kMaxPasses; ++pass)
		{
			bool anySplit = false;
			std::vector<float> next;
			next.reserve(soup.size() + soup.size() / 4U);

			for (std::size_t t = 0U; t + 8U < soup.size(); t += 9U)
			{
				const float ax = soup[t];
				const float ay = soup[t + 1U];
				const float az = soup[t + 2U];
				const float bx = soup[t + 3U];
				const float by = soup[t + 4U];
				const float bz = soup[t + 5U];
				const float cx = soup[t + 6U];
				const float cy = soup[t + 7U];
				const float cz = soup[t + 8U];

				const double lab2 = triEdgeLen2(ax, ay, az, bx, by, bz);
				const double lbc2 = triEdgeLen2(bx, by, bz, cx, cy, cz);
				const double lca2 = triEdgeLen2(cx, cy, cz, ax, ay, az);
				double longest2 = lab2;
				int longest = 0;
				if (lbc2 > longest2)
				{
					longest2 = lbc2;
					longest = 1;
				}
				if (lca2 > longest2)
				{
					longest2 = lca2;
					longest = 2;
				}

				if (longest2 <= maxE2)
				{
					appendTri9(next, ax, ay, az, bx, by, bz, cx, cy, cz);
					continue;
				}

				anySplit = true;
				const float mx =
					(longest == 0) ? 0.5f * (ax + bx) : ((longest == 1) ? 0.5f * (bx + cx) : 0.5f * (cx + ax));
				const float my =
					(longest == 0) ? 0.5f * (ay + by) : ((longest == 1) ? 0.5f * (by + cy) : 0.5f * (cy + ay));
				const float mz =
					(longest == 0) ? 0.5f * (az + bz) : ((longest == 1) ? 0.5f * (bz + cz) : 0.5f * (cz + az));

				if (longest == 0)
				{
					appendTri9(next, ax, ay, az, mx, my, mz, cx, cy, cz);
					appendTri9(next, mx, my, mz, bx, by, bz, cx, cy, cz);
				}
				else if (longest == 1)
				{
					appendTri9(next, ax, ay, az, bx, by, bz, mx, my, mz);
					appendTri9(next, ax, ay, az, mx, my, mz, cx, cy, cz);
				}
				else
				{
					appendTri9(next, ax, ay, az, bx, by, bz, mx, my, mz);
					appendTri9(next, mx, my, mz, bx, by, bz, cx, cy, cz);
				}
			}

			if (next.size() / 9U > kMaxTris)
			{
				break;
			}
			soup.swap(next);
			if (!anySplit)
			{
				break;
			}
		}

		if (soup.empty())
		{
			if (errMsg)
			{
				*errMsg = "refineTriangleSoupToMaxEdge produced empty soup";
			}
			return false;
		}
		return true;
	}
	catch (const std::bad_alloc&)
	{
		if (errMsg)
		{
			*errMsg = "out of memory while refining to target edge length; try a larger edge";
		}
		return false;
	}
}

bool discretizeShapeHandleToMesh(const ShapeHandle& handle, const MeshDiscretizeParams& params,
								 std::vector<float>& soup, MeshDiscretizeReport& report, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(handle, &shape) || shape.IsNull())
	{
		if (errMsg)
			*errMsg = "null ShapeHandle";
		return false;
	}
	return discretizeShapeToMesh(shape, params, soup, report, errMsg);
}

bool discretizeWireToMesh(const TopoDS_Wire& wire, const MeshDiscretizeParams& params, std::vector<float>& soup,
						  std::string* errMsg)
{
	Polyline3d poly;
	if (!discretizeWire(wire, params.tessellate, poly, errMsg))
	{
		return false;
	}
	return discretizePolylineToMesh(poly, params, soup, errMsg);
}

bool discretizePolylineToMesh(const Polyline3d& polyline, const MeshDiscretizeParams& params, std::vector<float>& soup,
							  std::string* errMsg)
{
	soup.clear();
	MeshDiscretizeParams p = params;
	applyQualityPreset(p);
	switch (p.mode)
	{
	case MeshDiscretizeMode::WireRibbonMesh:
		return buildRibbonAlongPolyline(polyline, p, soup, errMsg);
	case MeshDiscretizeMode::WireTubeMesh:
	default:
		return buildTubeAlongPolyline(polyline, p, soup, errMsg);
	}
}

bool tessellateStepFileToMesh(const std::string& pathLocal, const MeshDiscretizeParams& params,
							  std::vector<float>& soup, MeshDiscretizeReport& report, std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!readStepShape(pathLocal, shape, errMsg))
	{
		return false;
	}
	return discretizeShapeToMesh(shape, params, soup, report, errMsg);
}

bool remeshTriangleSoup(const std::vector<float>& inSoup, const MeshDiscretizeParams& params,
						std::vector<float>& outSoup, std::string* errMsg)
{
	(void)params;
	outSoup = inSoup;
	if (errMsg)
	{
		*errMsg = "RemeshSoup not implemented";
	}
	return false;
}

} // namespace geoalgo
