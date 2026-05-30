#include "detail/OccIncludes.h"

#include "Discretize.h"
#include "MeshDiscretize.h"
#include "ShapeIo.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace geoalgo
{
namespace
{

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

bool discretizeFaceUVGridInternal(
	const TopoDS_Face& face,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
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
	gp_Vec v(
		static_cast<double>(p1[0] - p0[0]),
		static_cast<double>(p1[1] - p0[1]),
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

bool buildTubeAlongPolyline(const Polyline3d& poly, const MeshDiscretizeParams& params, std::vector<float>& soup, std::string* errMsg)
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

	auto pointAt = [&](std::size_t i) {
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

bool buildRibbonAlongPolyline(const Polyline3d& poly, const MeshDiscretizeParams& params, std::vector<float>& soup, std::string* errMsg)
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
	auto pointAt = [&](std::size_t i) {
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

bool discretizeFaceToMesh(
	const TopoDS_Face& face,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
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

bool discretizeShapeToMesh(
	const TopoDS_Shape& shape,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	MeshDiscretizeReport& report,
	std::string* errMsg)
{
	soup.clear();
	MeshDiscretizeParams p = params;
	applyQualityPreset(p);
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
		if (!discretizeShapeToSoup(shape, p.tessellate, soup, errMsg))
		{
			return false;
		}
		break;
	}
	fillMeshReport(soup, report);
	return !soup.empty();
}

bool discretizeWireToMesh(
	const TopoDS_Wire& wire,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	std::string* errMsg)
{
	Polyline3d poly;
	if (!discretizeWire(wire, params.tessellate, poly, errMsg))
	{
		return false;
	}
	return discretizePolylineToMesh(poly, params, soup, errMsg);
}

bool discretizePolylineToMesh(
	const Polyline3d& polyline,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
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

bool tessellateStepFileToMesh(
	const std::string& pathLocal,
	const MeshDiscretizeParams& params,
	std::vector<float>& soup,
	MeshDiscretizeReport& report,
	std::string* errMsg)
{
	TopoDS_Shape shape;
	if (!readStepShape(pathLocal, shape, errMsg))
	{
		return false;
	}
	return discretizeShapeToMesh(shape, params, soup, report, errMsg);
}

bool remeshTriangleSoup(
	const std::vector<float>& inSoup,
	const MeshDiscretizeParams& params,
	std::vector<float>& outSoup,
	std::string* errMsg)
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
