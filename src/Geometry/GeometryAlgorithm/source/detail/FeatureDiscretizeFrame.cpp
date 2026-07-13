#include "detail/FeatureDiscretizeFrame.h"

#include "ShapeQuery.h"
#include "detail/OccIncludes.h"

#include <Precision.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>

#include <cmath>
#include <limits>

namespace geoalgo
{
namespace detail
{
namespace
{

void normalizeVec3(Vec3d& v)
{
	const double len = std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
	if (len > 1e-12)
	{
		v.x /= len;
		v.y /= len;
		v.z /= len;
	}
}

Vec3d vecFromGp(const gp_Vec& v)
{
	return Vec3d{v.X(), v.Y(), v.Z()};
}

bool computeFaceNormalAtUv(
	const TopoDS_Face& face,
	const Handle(Geom_Surface)& surf,
	const TopLoc_Location& faceLoc,
	const Standard_Real u,
	const Standard_Real v,
	gp_Vec& outNormal)
{
	gp_Pnt pOnSurf;
	gp_Vec du;
	gp_Vec dv;
	surf->D1(u, v, pOnSurf, du, dv);
	gp_Vec nLocal = du.Crossed(dv);
	const double mag2 = nLocal.SquareMagnitude();
	if (mag2 <= Precision::SquareConfusion())
	{
		return false;
	}
	nLocal /= std::sqrt(mag2);
	if (face.Orientation() == TopAbs_REVERSED)
	{
		nLocal.Reverse();
	}
	const gp_Trsf& toGlobal = faceLoc.Transformation();
	nLocal.Transform(toGlobal);
	outNormal = nLocal;
	if (outNormal.Magnitude() > Precision::Confusion())
	{
		outNormal.Normalize();
	}
	return true;
}

bool tryFaceNormalAtPoint(
	const TopoDS_Face& face,
	const gp_Pnt& ptWorld,
	double& inOutBestDist,
	gp_Vec& inOutBestNormal)
{
	TopLoc_Location faceLoc;
	const Handle(Geom_Surface) surf = BRep_Tool::Surface(face, faceLoc);
	if (surf.IsNull())
	{
		return false;
	}

	const gp_Trsf& toGlobal = faceLoc.Transformation();
	gp_Trsf toLocal = toGlobal;
	toLocal.Invert();

	gp_Pnt ptLocal = ptWorld;
	ptLocal.Transform(toLocal);

	GeomAPI_ProjectPointOnSurf proj(ptLocal, surf);
	if (!proj.IsDone() || proj.NbPoints() < 1)
	{
		return false;
	}

	Standard_Real u = 0.0;
	Standard_Real v = 0.0;
	proj.LowerDistanceParameters(u, v);

	gp_Pnt pOnSurf;
	surf->D0(u, v, pOnSurf);
	pOnSurf.Transform(toGlobal);
	const double dist = ptWorld.Distance(pOnSurf);

	gp_Vec n;
	if (!computeFaceNormalAtUv(face, surf, faceLoc, u, v, n))
	{
		return false;
	}

	if (dist < inOutBestDist)
	{
		inOutBestDist = dist;
		inOutBestNormal = n;
		return true;
	}
	return false;
}

Vec3d fallbackNormalFromTangent(const Vec3d& tangent)
{
	Vec3d up{0.0, 0.0, 1.0};
	Vec3d nrm{
		tangent.y * up.z - tangent.z * up.y,
		tangent.z * up.x - tangent.x * up.z,
		tangent.x * up.y - tangent.y * up.x};
	normalizeVec3(nrm);
	if (std::abs(nrm.x) + std::abs(nrm.y) + std::abs(nrm.z) < 1e-9)
	{
		up = Vec3d{0.0, 1.0, 0.0};
		nrm.x = tangent.y * up.z - tangent.z * up.y;
		nrm.y = tangent.z * up.x - tangent.x * up.z;
		nrm.z = tangent.x * up.y - tangent.y * up.x;
		normalizeVec3(nrm);
	}
	return nrm;
}

bool isSegmentStart(std::size_t index, const std::vector<std::size_t>* segmentEndExclusive)
{
	if (index == 0U)
	{
		return true;
	}
	if (!segmentEndExclusive)
	{
		return false;
	}
	for (const std::size_t end : *segmentEndExclusive)
	{
		if (index == end)
		{
			return true;
		}
	}
	return false;
}

} // namespace

bool bestFaceNormalAtPoint(const std::vector<TopoDS_Face>& faces, const Point3d& pt, Vec3d& outNormal)
{
	const gp_Pnt gpPt(pt.x, pt.y, pt.z);
	const double maxDist = Precision::Confusion() * 100.0 * 10.0;

	double bestDist = std::numeric_limits<double>::max();
	gp_Vec bestNormal;
	bool found = false;

	for (const TopoDS_Face& face : faces)
	{
		found = tryFaceNormalAtPoint(face, gpPt, bestDist, bestNormal) || found;
	}

	if (!found || bestDist > maxDist)
	{
		return false;
	}

	outNormal = vecFromGp(bestNormal);
	return true;
}

Vec3d chordTangentAt(
	const std::vector<Point3d>& pts,
	std::size_t index,
	bool pathClosed,
	const std::vector<std::size_t>* segmentEndExclusive)
{
	Vec3d tan{};
	if (pts.size() < 2U)
	{
		return tan;
	}
	if (index + 1U < pts.size() && !isSegmentStart(index + 1U, segmentEndExclusive))
	{
		tan.x = pts[index + 1U].x - pts[index].x;
		tan.y = pts[index + 1U].y - pts[index].y;
		tan.z = pts[index + 1U].z - pts[index].z;
	}
	else if (pathClosed && pts.size() > 1U && index + 1U == pts.size())
	{
		tan.x = pts[0].x - pts[index].x;
		tan.y = pts[0].y - pts[index].y;
		tan.z = pts[0].z - pts[index].z;
	}
	else if (index > 0U && !isSegmentStart(index, segmentEndExclusive))
	{
		tan.x = pts[index].x - pts[index - 1U].x;
		tan.y = pts[index].y - pts[index - 1U].y;
		tan.z = pts[index].z - pts[index - 1U].z;
	}
	normalizeVec3(tan);
	return tan;
}

Vec3d storedFaceNormal(
	const std::vector<TopoDS_Face>& faces,
	const Point3d& pt,
	FaceNormalConvention convention,
	bool* found)
{
	Vec3d normal{};
	const bool ok = bestFaceNormalAtPoint(faces, pt, normal);
	if (found)
	{
		*found = ok;
	}
	if (!ok)
	{
		return normal;
	}
	if (convention == FaceNormalConvention::LineReverseFace)
	{
		normal.x = -normal.x;
		normal.y = -normal.y;
		normal.z = -normal.z;
	}
	return normal;
}

void assignPathChordTangents(
	RawPath& path,
	bool pathClosed,
	bool outputTangent,
	const std::vector<std::size_t>* segmentEndExclusive,
	bool preserveExisting)
{
	if (!outputTangent || path.points.size() < 2U)
	{
		return;
	}
	std::vector<Point3d> pts;
	pts.reserve(path.points.size());
	for (const RawPathPoint& rp : path.points)
	{
		pts.push_back(rp.positionMm);
	}
	for (std::size_t i = 0; i < path.points.size(); ++i)
	{
		if (preserveExisting && path.points[i].hasTangent)
		{
			continue;
		}
		const Vec3d tan = chordTangentAt(pts, i, pathClosed, segmentEndExclusive);
		if (std::abs(tan.x) + std::abs(tan.y) + std::abs(tan.z) > 1e-9)
		{
			path.points[i].tangent = tan;
			path.points[i].hasTangent = true;
		}
	}
}

void appendPolylineToRawPath(
	const Polyline3d& poly,
	RawPath& out,
	const DiscretizeParams& disc,
	bool computeFrame,
	const PolylineFrameContext& frameCtx)
{
	const std::size_t n = poly.xyz.size() / 3U;
	if (n == 0U)
	{
		return;
	}
	std::vector<Point3d> pts;
	pts.reserve(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		pts.push_back(Point3d{poly.xyz[b], poly.xyz[b + 1U], poly.xyz[b + 2U]});
	}
	for (std::size_t i = 0; i < pts.size(); ++i)
	{
		RawPathPoint rp;
		rp.positionMm = pts[i];
		if (computeFrame && disc.outputTangent && pts.size() >= 2U)
		{
			const Vec3d tan = chordTangentAt(pts, i, frameCtx.pathClosed, nullptr);
			if (std::abs(tan.x) + std::abs(tan.y) + std::abs(tan.z) > 1e-9)
			{
				rp.tangent = tan;
				rp.hasTangent = true;
			}
		}
		if (computeFrame && disc.outputNormal)
		{
			bool hasFaceNormal = false;
			if (frameCtx.faces && !frameCtx.faces->empty())
			{
				rp.normal = storedFaceNormal(*frameCtx.faces, pts[i], frameCtx.normalConvention, &hasFaceNormal);
			}
			if (hasFaceNormal)
			{
				rp.hasNormal = true;
			}
			else if (rp.hasTangent)
			{
				// 无关联面时退化为切向叉乘参考轴
				rp.normal = fallbackNormalFromTangent(rp.tangent);
				rp.hasNormal = true;
			}
		}
		out.points.push_back(rp);
	}
}

std::vector<TopoDS_Face> collectContextFaces(
	const TopoDS_Shape& shape,
	const std::vector<int>& edgeIndices,
	const std::vector<int>& faceIndices,
	std::string* errMsg)
{
	std::vector<TopoDS_Face> faces;
	if (!faceIndices.empty())
	{
		faces.reserve(faceIndices.size());
		for (const int faceIndex : faceIndices)
		{
			TopoDS_Face face;
			if (!shapeFaceAtIndex(shape, faceIndex, face, errMsg))
			{
				return {};
			}
			faces.push_back(face);
		}
		return faces;
	}
	if (edgeIndices.empty())
	{
		return faces;
	}

	TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
	TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);
	for (const int edgeIndex : edgeIndices)
	{
		TopoDS_Edge edge;
		if (!shapeEdgeAtIndex(shape, edgeIndex, edge, errMsg))
		{
			return {};
		}
		if (!edgeFaceMap.Contains(edge))
		{
			continue;
		}
		const TopTools_ListOfShape& adjacentFaces = edgeFaceMap.FindFromKey(edge);
		for (TopTools_ListIteratorOfListOfShape it(adjacentFaces); it.More(); it.Next())
		{
			faces.push_back(TopoDS::Face(it.Value()));
		}
	}
	return faces;
}

} // namespace detail
} // namespace geoalgo
