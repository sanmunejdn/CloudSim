#include "detail/OccIncludes.h"

#include "Discretize.h"
#include "FeatureSpec.h"
#include "Intersection.h"
#include "ShapeIo.h"
#include "ShapeQuery.h"
#include "WireOps.h"

#include <json.hpp>

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <Bnd_Box.hxx>
#include <GProp_GProps.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopTools_IndexedDataMapOfShapeListOfShape.hxx>

#include <algorithm>
#include <cmath>
#include <sstream>

namespace geoalgo
{
namespace
{

constexpr double kPi = 3.14159265358979323846;

TessellateParams toTessellate(const DiscretizeParams& p)
{
	TessellateParams t;
	t.linearDeflectionMm = p.linearDeflectionMm;
	t.linearDeflectionRelative = true;
	return t;
}

Vec3d vecFromGp(const gp_Dir& d)
{
	return Vec3d{d.X(), d.Y(), d.Z()};
}

Vec3d vecFromGp(const gp_Vec& v)
{
	return Vec3d{v.X(), v.Y(), v.Z()};
}

Point3d pointFromGp(const gp_Pnt& p)
{
	return Point3d{p.X(), p.Y(), p.Z()};
}

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

void appendPolylineToRawPath(
	const Polyline3d& poly,
	RawPath& out,
	const DiscretizeParams& disc,
	bool computeFrame)
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
			Vec3d tan{};
			if (i + 1U < pts.size())
			{
				tan.x = pts[i + 1U].x - pts[i].x;
				tan.y = pts[i + 1U].y - pts[i].y;
				tan.z = pts[i + 1U].z - pts[i].z;
			}
			else if (out.closed && pts.size() > 1U)
			{
				tan.x = pts[0].x - pts[i].x;
				tan.y = pts[0].y - pts[i].y;
				tan.z = pts[0].z - pts[i].z;
			}
			else if (i > 0U)
			{
				tan.x = pts[i].x - pts[i - 1U].x;
				tan.y = pts[i].y - pts[i - 1U].y;
				tan.z = pts[i].z - pts[i - 1U].z;
			}
			normalizeVec3(tan);
			rp.tangent = tan;
			rp.hasTangent = true;
			if (disc.outputNormal)
			{
				Vec3d up{0.0, 0.0, 1.0};
				Vec3d nrm{
					tan.y * up.z - tan.z * up.y,
					tan.z * up.x - tan.x * up.z,
					tan.x * up.y - tan.y * up.x};
				normalizeVec3(nrm);
				if (std::abs(nrm.x) + std::abs(nrm.y) + std::abs(nrm.z) < 1e-9)
				{
					up = Vec3d{0.0, 1.0, 0.0};
					nrm.x = tan.y * up.z - tan.z * up.y;
					nrm.y = tan.z * up.x - tan.x * up.z;
					nrm.z = tan.x * up.y - tan.y * up.x;
					normalizeVec3(nrm);
				}
				rp.normal = nrm;
				rp.hasNormal = true;
			}
		}
		out.points.push_back(rp);
	}
}

bool resampleRawPathByStep(RawPath& path, double stepMm)
{
	if (path.points.size() < 2U || stepMm <= 0.0)
	{
		return true;
	}
	std::vector<double> segLen;
	double total = 0.0;
	for (std::size_t i = 1; i < path.points.size(); ++i)
	{
		const auto& a = path.points[i - 1U].positionMm;
		const auto& b = path.points[i].positionMm;
		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double dz = b.z - a.z;
		const double len = std::sqrt(dx * dx + dy * dy + dz * dz);
		segLen.push_back(len);
		total += len;
	}
	if (path.closed && path.points.size() > 1U)
	{
		const auto& a = path.points.back().positionMm;
		const auto& b = path.points.front().positionMm;
		const double dx = b.x - a.x;
		const double dy = b.y - a.y;
		const double dz = b.z - a.z;
		segLen.push_back(std::sqrt(dx * dx + dy * dy + dz * dz));
		total += segLen.back();
	}
	if (total < stepMm)
	{
		return true;
	}
	const int sampleCount = std::max(2, static_cast<int>(std::ceil(total / stepMm)) + 1);
	RawPath resampled;
	resampled.sourceSpec = path.sourceSpec;
	resampled.closed = path.closed;
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
		const std::size_t i0 = seg % path.points.size();
		const std::size_t i1 = (i0 + 1U) % path.points.size();
		const auto& p0 = path.points[i0].positionMm;
		const auto& p1 = path.points[i1].positionMm;
		RawPathPoint rp;
		rp.positionMm.x = p0.x + (p1.x - p0.x) * local;
		rp.positionMm.y = p0.y + (p1.y - p0.y) * local;
		rp.positionMm.z = p0.z + (p1.z - p0.z) * local;
		if (path.points[i0].hasTangent)
		{
			rp.tangent = path.points[i0].tangent;
			rp.hasTangent = true;
		}
		if (path.points[i0].hasNormal)
		{
			rp.normal = path.points[i0].normal;
			rp.hasNormal = true;
		}
		resampled.points.push_back(rp);
	}
	path = std::move(resampled);
	return true;
}

bool loadShapeForSpec(const FeatureSpec& spec, TopoDS_Shape& shape, std::string* errMsg)
{
	if (spec.workpiece.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "workpiece.stepPathUtf8 is empty";
		}
		return false;
	}
	return readStepShape(spec.workpiece.stepPathUtf8, shape, errMsg);
}

bool discretizeEdgeChainInternal(
	const TopoDS_Shape& shape,
	const FeatureSpec& spec,
	RawPath& out,
	std::string* errMsg)
{
	if (spec.refs.edgeIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "EdgeChain requires edgeIndices";
		}
		return false;
	}
	const TessellateParams disc = toTessellate(spec.discretize);
	if (spec.refs.edgeIndices.size() == 1)
	{
		TopoDS_Edge edge;
		if (!shapeEdgeAtIndex(shape, spec.refs.edgeIndices[0], edge, errMsg))
		{
			return false;
		}
		Polyline3d poly;
		if (!discretizeEdge(edge, disc, poly, errMsg))
		{
			return false;
		}
		appendPolylineToRawPath(poly, out, spec.discretize, true);
		return !out.points.empty();
	}
	Polyline3d poly;
	if (!fuseStepEdgesToPolyline(spec.workpiece.stepPathUtf8, spec.refs.edgeIndices, disc, poly, errMsg))
	{
		return false;
	}
	appendPolylineToRawPath(poly, out, spec.discretize, true);
	return !out.points.empty();
}

bool discretizeFaceBoundaryInternal(
	const TopoDS_Shape& shape,
	const FeatureSpec& spec,
	RawPath& out,
	std::string* errMsg)
{
	if (spec.refs.faceIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "FaceBoundary requires faceIndices";
		}
		return false;
	}
	TopoDS_Face face;
	if (!shapeFaceAtIndex(shape, spec.refs.faceIndices[0], face, errMsg))
	{
		return false;
	}
	const TopoDS_Wire wire = BRepTools::OuterWire(face);
	if (wire.IsNull())
	{
		if (errMsg)
		{
			*errMsg = "face has no outer wire";
		}
		return false;
	}
	Polyline3d poly;
	const TessellateParams disc = toTessellate(spec.discretize);
	if (!discretizeWire(wire, disc, poly, errMsg))
	{
		return false;
	}
	appendPolylineToRawPath(poly, out, spec.discretize, true);
	out.closed = true;
	return !out.points.empty();
}

bool curvesToRawPath(
	const IntersectionResult& result,
	const FeatureSpec& spec,
	RawPath& out,
	std::string* errMsg)
{
	if (!result.curves.empty())
	{
		for (const Polyline3d& poly : result.curves)
		{
			appendPolylineToRawPath(poly, out, spec.discretize, true);
		}
	}
	else if (!result.points.empty())
	{
		for (const IntersectionHit& hit : result.points)
		{
			RawPathPoint rp;
			rp.positionMm = hit.positionMm;
			out.points.push_back(rp);
		}
	}
	if (out.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "intersection produced no geometry";
		}
		return false;
	}
	return true;
}

bool discretizeFaceIntersectionInternal(
	const TopoDS_Shape& shape,
	const FeatureSpec& spec,
	RawPath& out,
	std::string* errMsg)
{
	if (spec.refs.faceIndices.size() < 2U)
	{
		if (errMsg)
		{
			*errMsg = "FaceIntersection requires two faceIndices";
		}
		return false;
	}
	TopoDS_Face f1;
	TopoDS_Face f2;
	if (!shapeFaceAtIndex(shape, spec.refs.faceIndices[0], f1, errMsg)
		|| !shapeFaceAtIndex(shape, spec.refs.faceIndices[1], f2, errMsg))
	{
		return false;
	}
	IntersectionParams ip;
	ip.discretizeCurves = true;
	ip.curveDisc = toTessellate(spec.discretize);
	IntersectionResult result;
	if (!intersectFaces(f1, f2, ip, result, errMsg))
	{
		return false;
	}
	return curvesToRawPath(result, spec, out, errMsg);
}

bool discretizeFaceOffsetCurveInternal(
	const TopoDS_Shape& shape,
	const FeatureSpec& spec,
	RawPath& out,
	std::string* errMsg)
{
	if (spec.refs.faceIndices.empty() || spec.refs.edgeIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "FaceOffsetCurve requires faceIndices and edgeIndices";
		}
		return false;
	}
	TopoDS_Face face;
	TopoDS_Edge edge;
	if (!shapeFaceAtIndex(shape, spec.refs.faceIndices[0], face, errMsg)
		|| !shapeEdgeAtIndex(shape, spec.refs.edgeIndices[0], edge, errMsg))
	{
		return false;
	}
	Polyline3d poly;
	const TessellateParams disc = toTessellate(spec.discretize);
	if (!discretizeEdge(edge, disc, poly, errMsg))
	{
		return false;
	}
	const BRepAdaptor_Surface surf(face);
	const double offset = spec.refs.offsetMm;
	const std::size_t n = poly.xyz.size() / 3U;
	for (std::size_t i = 0; i < n; ++i)
	{
		const std::size_t b = i * 3U;
		gp_Pnt p(poly.xyz[b], poly.xyz[b + 1U], poly.xyz[b + 2U]);
		GeomAPI_ProjectPointOnSurf proj(p, BRep_Tool::Surface(face));
		if (proj.NbPoints() < 1)
		{
			continue;
		}
		Standard_Real u = 0.0;
		Standard_Real v = 0.0;
		proj.LowerDistanceParameters(u, v);
		gp_Pnt onS;
		gp_Vec du;
		gp_Vec dv;
		surf.D1(u, v, onS, du, dv);
		gp_Vec nrm = du.Crossed(dv);
		if (face.Orientation() == TopAbs_REVERSED)
		{
			nrm.Reverse();
		}
		if (nrm.Magnitude() < 1e-12)
		{
			continue;
		}
		nrm.Normalize();
		p.Translate(nrm.Multiplied(offset));
		RawPathPoint rp;
		rp.positionMm = pointFromGp(p);
		rp.normal = vecFromGp(gp_Dir(nrm));
		rp.hasNormal = true;
		out.points.push_back(rp);
	}
	return !out.points.empty();
}

bool discretizeFaceUVGridInternal(
	const TopoDS_Shape& shape,
	const FeatureSpec& spec,
	RawPath& out,
	std::string* errMsg)
{
	if (spec.refs.faceIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "FaceUVGrid requires faceIndices";
		}
		return false;
	}
	TopoDS_Face face;
	if (!shapeFaceAtIndex(shape, spec.refs.faceIndices[0], face, errMsg))
	{
		return false;
	}
	const BRepAdaptor_Surface surf(face);
	const double u0 = surf.FirstUParameter();
	const double u1 = surf.LastUParameter();
	const double v0 = surf.FirstVParameter();
	const double v1 = surf.LastVParameter();
	const int nu = std::max(2, spec.refs.uvCountU);
	const int nv = std::max(2, spec.refs.uvCountV);
	const double angleRad = spec.refs.gridAngleDeg * kPi / 180.0;
	const double cosA = std::cos(angleRad);
	const double sinA = std::sin(angleRad);

	for (int iv = 0; iv < nv; ++iv)
	{
		const double fv = static_cast<double>(iv) / static_cast<double>(nv - 1);
		const double vBase = v0 + (v1 - v0) * fv;
		for (int iu = 0; iu < nu; ++iu)
		{
			const double fu = static_cast<double>(iu) / static_cast<double>(nu - 1);
			const double uBase = u0 + (u1 - u0) * fu;
			const double u = uBase * cosA - vBase * sinA;
			const double v = uBase * sinA + vBase * cosA;
			const gp_Pnt p = surf.Value(u, v);
			gp_Vec du;
			gp_Vec dv;
			gp_Pnt onS;
			surf.D1(u, v, onS, du, dv);
			gp_Vec nrm = du.Crossed(dv);
			if (face.Orientation() == TopAbs_REVERSED)
			{
				nrm.Reverse();
			}
			RawPathPoint rp;
			rp.positionMm = pointFromGp(p);
			if (nrm.Magnitude() > 1e-12)
			{
				nrm.Normalize();
				rp.normal = vecFromGp(gp_Dir(nrm));
				rp.hasNormal = true;
			}
			out.points.push_back(rp);
		}
	}
	return !out.points.empty();
}

bool discretizeSyntheticPolylineInternal(const FeatureSpec& spec, RawPath& out, std::string* errMsg)
{
	const auto& xyz = spec.refs.polylineXyz;
	if (xyz.size() < 6U)
	{
		if (errMsg)
		{
			*errMsg = "SyntheticPolyline requires at least 2 points";
		}
		return false;
	}
	Polyline3d poly;
	poly.xyz = xyz;
	appendPolylineToRawPath(poly, out, spec.discretize, true);
	return true;
}

double edgeLengthMm(const TopoDS_Edge& edge)
{
	GProp_GProps props;
	BRepGProp::LinearProperties(edge, props);
	return props.Mass();
}

double faceAreaMm2(const TopoDS_Face& face)
{
	GProp_GProps props;
	BRepGProp::SurfaceProperties(face, props);
	return props.Mass();
}

double edgeDihedralDeg(const TopoDS_Shape& shape, const TopoDS_Edge& edge)
{
	TopTools_IndexedDataMapOfShapeListOfShape edgeFaceMap;
	TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, edgeFaceMap);
	const TopTools_ListOfShape& faces = edgeFaceMap.FindFromKey(edge);
	if (faces.Extent() < 2)
	{
		return 0.0;
	}
	TopTools_ListIteratorOfListOfShape it(faces);
	const TopoDS_Face f1 = TopoDS::Face(it.Value());
	it.Next();
	const TopoDS_Face f2 = TopoDS::Face(it.Value());
	BRepAdaptor_Curve curve(edge);
	BRepAdaptor_Surface s1(f1);
	BRepAdaptor_Surface s2(f2);
	(void)s1;
	(void)s2;
	// 二面角启发式：邻面法向夹角
	GeomAPI_ProjectPointOnCurve proj;
	(void)proj;
	const double mid = (curve.FirstParameter() + curve.LastParameter()) * 0.5;
	gp_Pnt pm;
	gp_Vec tan;
	curve.D1(mid, pm, tan);
	if (tan.Magnitude() < 1e-12)
	{
		return 0.0;
	}
	gp_Vec n1;
	gp_Vec n2;
	{
		GeomAPI_ProjectPointOnSurf ps1(pm, BRep_Tool::Surface(f1));
		if (ps1.NbPoints() < 1)
		{
			return 0.0;
		}
		Standard_Real uu = 0.0;
		Standard_Real vv = 0.0;
		ps1.LowerDistanceParameters(uu, vv);
		gp_Pnt dummy;
		gp_Vec du;
		gp_Vec dv;
		s1.D1(uu, vv, dummy, du, dv);
		n1 = du.Crossed(dv);
		if (f1.Orientation() == TopAbs_REVERSED)
		{
			n1.Reverse();
		}
	}
	{
		GeomAPI_ProjectPointOnSurf ps2(pm, BRep_Tool::Surface(f2));
		if (ps2.NbPoints() < 1)
		{
			return 0.0;
		}
		Standard_Real uu = 0.0;
		Standard_Real vv = 0.0;
		ps2.LowerDistanceParameters(uu, vv);
		gp_Pnt dummy;
		gp_Vec du;
		gp_Vec dv;
		s2.D1(uu, vv, dummy, du, dv);
		n2 = du.Crossed(dv);
		if (f2.Orientation() == TopAbs_REVERSED)
		{
			n2.Reverse();
		}
	}
	if (n1.Magnitude() < 1e-12 || n2.Magnitude() < 1e-12)
	{
		return 0.0;
	}
	n1.Normalize();
	n2.Normalize();
	const double dot = std::max(-1.0, std::min(1.0, n1.Dot(n2)));
	return std::acos(dot) * 180.0 / kPi;
}

void writeWorkpieceJson(nlohmann::json& j, const WorkpieceRef& wp)
{
	j["backendIdUtf8"] = wp.backendIdUtf8;
	j["stepPathUtf8"] = wp.stepPathUtf8;
	if (!wp.frameId.empty() && wp.frameId != "workpiece")
	{
		j["frameId"] = wp.frameId;
	}
}

void writeRefsJson(nlohmann::json& j, const FeatureRefs& refs)
{
	if (!refs.edgeIndices.empty())
	{
		j["edgeIndices"] = refs.edgeIndices;
	}
	if (!refs.faceIndices.empty())
	{
		j["faceIndices"] = refs.faceIndices;
	}
	if (refs.offsetMm != 0.0)
	{
		j["offsetMm"] = refs.offsetMm;
	}
	if (refs.uvCountU != 32)
	{
		j["uvCountU"] = refs.uvCountU;
	}
	if (refs.uvCountV != 32)
	{
		j["uvCountV"] = refs.uvCountV;
	}
	if (refs.gridAngleDeg != 0.0)
	{
		j["gridAngleDeg"] = refs.gridAngleDeg;
	}
	if (!refs.polylineXyz.empty())
	{
		j["polylineXyz"] = refs.polylineXyz;
	}
	if (!refs.children.empty())
	{
		nlohmann::json kids = nlohmann::json::array();
		for (const FeatureSpec& child : refs.children)
		{
			kids.push_back(nlohmann::json::parse(featureSpecToJson(child)));
		}
		j["children"] = kids;
	}
}

bool readWorkpieceJson(const nlohmann::json& j, WorkpieceRef& wp, std::string* errMsg)
{
	if (!j.is_object())
	{
		if (errMsg)
		{
			*errMsg = "workpiece must be object";
		}
		return false;
	}
	wp.backendIdUtf8 = j.value("backendIdUtf8", "");
	wp.stepPathUtf8 = j.value("stepPathUtf8", "");
	wp.frameId = j.value("frameId", "workpiece");
	return true;
}

bool readRefsJson(const nlohmann::json& j, FeatureRefs& refs, std::string* errMsg)
{
	if (!j.is_object())
	{
		return true;
	}
	if (j.contains("edgeIndices") && j["edgeIndices"].is_array())
	{
		for (const auto& v : j["edgeIndices"])
		{
			refs.edgeIndices.push_back(v.get<int>());
		}
	}
	if (j.contains("faceIndices") && j["faceIndices"].is_array())
	{
		for (const auto& v : j["faceIndices"])
		{
			refs.faceIndices.push_back(v.get<int>());
		}
	}
	refs.offsetMm = j.value("offsetMm", 0.0);
	refs.uvCountU = j.value("uvCountU", 32);
	refs.uvCountV = j.value("uvCountV", 32);
	refs.gridAngleDeg = j.value("gridAngleDeg", 0.0);
	if (j.contains("polylineXyz") && j["polylineXyz"].is_array())
	{
		for (const auto& v : j["polylineXyz"])
		{
			refs.polylineXyz.push_back(static_cast<float>(v.get<double>()));
		}
	}
	if (j.contains("children") && j["children"].is_array())
	{
		for (const auto& childJ : j["children"])
		{
			FeatureSpec child;
			if (!featureSpecFromJson(childJ.dump(), child, errMsg))
			{
				return false;
			}
			refs.children.push_back(std::move(child));
		}
	}
	return true;
}

} // namespace

const char* featureKindToString(FeatureKind kind)
{
	switch (kind)
	{
	case FeatureKind::FaceBoundary:
		return "FaceBoundary";
	case FeatureKind::FaceIntersection:
		return "FaceIntersection";
	case FeatureKind::FaceOffsetCurve:
		return "FaceOffsetCurve";
	case FeatureKind::FaceUVGrid:
		return "FaceUVGrid";
	case FeatureKind::Composite:
		return "Composite";
	case FeatureKind::SyntheticPolyline:
		return "SyntheticPolyline";
	case FeatureKind::EdgeChain:
	default:
		return "EdgeChain";
	}
}

bool featureKindFromString(const std::string& s, FeatureKind& out)
{
	if (s == "FaceBoundary")
	{
		out = FeatureKind::FaceBoundary;
		return true;
	}
	if (s == "FaceIntersection")
	{
		out = FeatureKind::FaceIntersection;
		return true;
	}
	if (s == "FaceOffsetCurve")
	{
		out = FeatureKind::FaceOffsetCurve;
		return true;
	}
	if (s == "FaceUVGrid")
	{
		out = FeatureKind::FaceUVGrid;
		return true;
	}
	if (s == "Composite")
	{
		out = FeatureKind::Composite;
		return true;
	}
	if (s == "SyntheticPolyline")
	{
		out = FeatureKind::SyntheticPolyline;
		return true;
	}
	if (s == "EdgeChain")
	{
		out = FeatureKind::EdgeChain;
		return true;
	}
	return false;
}

bool validateFeatureSpec(const FeatureSpec& spec, std::string* errMsg)
{
	if (spec.schemaVersion != 1)
	{
		if (errMsg)
		{
			*errMsg = "unsupported schemaVersion";
		}
		return false;
	}
	if (spec.kind != FeatureKind::SyntheticPolyline && spec.workpiece.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "workpiece.stepPathUtf8 required";
		}
		return false;
	}
	switch (spec.kind)
	{
	case FeatureKind::EdgeChain:
		if (spec.refs.edgeIndices.empty())
		{
			if (errMsg)
			{
				*errMsg = "EdgeChain requires edgeIndices";
			}
			return false;
		}
		break;
	case FeatureKind::FaceBoundary:
	case FeatureKind::FaceUVGrid:
	case FeatureKind::FaceOffsetCurve:
		if (spec.refs.faceIndices.empty())
		{
			if (errMsg)
			{
				*errMsg = "face kind requires faceIndices";
			}
			return false;
		}
		break;
	case FeatureKind::FaceIntersection:
		if (spec.refs.faceIndices.size() < 2U)
		{
			if (errMsg)
			{
				*errMsg = "FaceIntersection requires two faces";
			}
			return false;
		}
		break;
	case FeatureKind::Composite:
		if (spec.refs.children.empty())
		{
			if (errMsg)
			{
				*errMsg = "Composite requires children";
			}
			return false;
		}
		for (const FeatureSpec& child : spec.refs.children)
		{
			if (!validateFeatureSpec(child, errMsg))
			{
				return false;
			}
		}
		break;
	case FeatureKind::SyntheticPolyline:
		if (spec.refs.polylineXyz.size() < 6U)
		{
			if (errMsg)
			{
				*errMsg = "SyntheticPolyline requires polylineXyz";
			}
			return false;
		}
		break;
	}
	return true;
}

bool validateFeatureSpecWithShape(const FeatureSpec& spec, std::string* errMsg)
{
	if (!validateFeatureSpec(spec, errMsg))
	{
		return false;
	}
	if (spec.kind == FeatureKind::SyntheticPolyline)
	{
		return true;
	}
	TopoDS_Shape shape;
	if (!loadShapeForSpec(spec, shape, errMsg))
	{
		return false;
	}
	const int edgeCount = shapeEdgeCount(shape);
	const int faceCount = shapeFaceCount(shape);
	for (int idx : spec.refs.edgeIndices)
	{
		if (idx < 0 || idx >= edgeCount)
		{
			if (errMsg)
			{
				*errMsg = "edge index out of range";
			}
			return false;
		}
	}
	for (int idx : spec.refs.faceIndices)
	{
		if (idx < 0 || idx >= faceCount)
		{
			if (errMsg)
			{
				*errMsg = "face index out of range";
			}
			return false;
		}
	}
	if (spec.kind == FeatureKind::Composite)
	{
		for (const FeatureSpec& child : spec.refs.children)
		{
			if (!validateFeatureSpecWithShape(child, errMsg))
			{
				return false;
			}
		}
	}
	return true;
}

bool discretizeFeature(const FeatureSpec& spec, RawPath& out, std::string* errMsg)
{
	if (!validateFeatureSpec(spec, errMsg))
	{
		return false;
	}
	out = RawPath{};
	out.sourceSpec = spec;
	out.closed = false;

	if (spec.kind == FeatureKind::SyntheticPolyline)
	{
		if (!discretizeSyntheticPolylineInternal(spec, out, errMsg))
		{
			return false;
		}
	}
	else if (spec.kind == FeatureKind::Composite)
	{
		for (const FeatureSpec& child : spec.refs.children)
		{
			RawPath part;
			if (!discretizeFeature(child, part, errMsg))
			{
				return false;
			}
			out.points.insert(out.points.end(), part.points.begin(), part.points.end());
		}
	}
	else
	{
		TopoDS_Shape shape;
		if (!loadShapeForSpec(spec, shape, errMsg))
		{
			return false;
		}
		bool ok = false;
		switch (spec.kind)
		{
		case FeatureKind::EdgeChain:
			ok = discretizeEdgeChainInternal(shape, spec, out, errMsg);
			break;
		case FeatureKind::FaceBoundary:
			ok = discretizeFaceBoundaryInternal(shape, spec, out, errMsg);
			break;
		case FeatureKind::FaceIntersection:
			ok = discretizeFaceIntersectionInternal(shape, spec, out, errMsg);
			break;
		case FeatureKind::FaceOffsetCurve:
			ok = discretizeFaceOffsetCurveInternal(shape, spec, out, errMsg);
			break;
		case FeatureKind::FaceUVGrid:
			ok = discretizeFaceUVGridInternal(shape, spec, out, errMsg);
			break;
		default:
			if (errMsg)
			{
				*errMsg = "unsupported feature kind";
			}
			return false;
		}
		if (!ok)
		{
			return false;
		}
	}

	if (spec.discretize.stepMm > 0.0 && spec.kind != FeatureKind::FaceUVGrid)
	{
		resampleRawPathByStep(out, spec.discretize.stepMm);
	}
	return !out.points.empty();
}

bool discretizeFeatures(const std::vector<FeatureSpec>& specs, std::vector<RawPath>& out, std::string* errMsg)
{
	out.clear();
	out.reserve(specs.size());
	for (const FeatureSpec& spec : specs)
	{
		RawPath path;
		if (!discretizeFeature(spec, path, errMsg))
		{
			return false;
		}
		out.push_back(std::move(path));
	}
	return true;
}

bool enumerateFeatureCatalog(const WorkpieceRef& workpiece, FeatureCatalog& out, std::string* errMsg)
{
	out = FeatureCatalog{};
	out.backendIdUtf8 = workpiece.backendIdUtf8;
	out.stepPathUtf8 = workpiece.stepPathUtf8;
	if (workpiece.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "stepPathUtf8 required for catalog";
		}
		return false;
	}
	TopoDS_Shape shape;
	if (!readStepShape(workpiece.stepPathUtf8, shape, errMsg))
	{
		return false;
	}
	int edgeIdx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++edgeIdx)
	{
		const TopoDS_Edge edge = TopoDS::Edge(exp.Current());
		const double len = edgeLengthMm(edge);
		const double dihedral = edgeDihedralDeg(shape, edge);
		FeatureCandidate c;
		c.candidateId = "edge_" + std::to_string(edgeIdx);
		c.refs.edgeIndices = {edgeIdx};
		c.lengthMm = len;
		c.dihedralDeg = dihedral;
		if (dihedral > 30.0 && dihedral < 150.0)
		{
			c.suggestedKind = FeatureKind::EdgeChain;
			c.summary = "焊缝候选边，长度约 " + std::to_string(static_cast<int>(len)) + "mm，二面角 "
				+ std::to_string(static_cast<int>(dihedral)) + "°";
		}
		else
		{
			c.suggestedKind = FeatureKind::EdgeChain;
			c.summary = "边，长度约 " + std::to_string(static_cast<int>(len)) + "mm";
		}
		out.candidates.push_back(std::move(c));
	}
	int faceIdx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++faceIdx)
	{
		const TopoDS_Face face = TopoDS::Face(exp.Current());
		const double area = faceAreaMm2(face);
		FeatureCandidate c;
		c.candidateId = "face_" + std::to_string(faceIdx);
		c.refs.faceIndices = {faceIdx};
		c.areaMm2 = area;
		if (area > 10000.0)
		{
			c.suggestedKind = FeatureKind::FaceUVGrid;
			c.summary = "大平面候选，面积约 " + std::to_string(static_cast<int>(area)) + "mm²";
		}
		else
		{
			c.suggestedKind = FeatureKind::FaceBoundary;
			c.summary = "面，面积约 " + std::to_string(static_cast<int>(area)) + "mm²，可用外轮廓涂胶";
		}
		out.candidates.push_back(std::move(c));
	}
	return true;
}

bool featureSpecFromJson(const std::string& jsonUtf8, FeatureSpec& out, std::string* errMsg)
{
	try
	{
		const nlohmann::json j = nlohmann::json::parse(jsonUtf8);
		out = FeatureSpec{};
		out.schemaVersion = j.value("schemaVersion", 1);
		out.featureId = j.value("featureId", "");
		if (j.contains("workpiece"))
		{
			if (!readWorkpieceJson(j["workpiece"], out.workpiece, errMsg))
			{
				return false;
			}
		}
		const std::string kindStr = j.value("kind", "EdgeChain");
		if (!featureKindFromString(kindStr, out.kind))
		{
			if (errMsg)
			{
				*errMsg = "unknown kind: " + kindStr;
			}
			return false;
		}
		if (j.contains("refs"))
		{
			if (!readRefsJson(j["refs"], out.refs, errMsg))
			{
				return false;
			}
		}
		if (j.contains("discretize") && j["discretize"].is_object())
		{
			const auto& d = j["discretize"];
			out.discretize.stepMm = d.value("stepMm", 2.0);
			out.discretize.linearDeflectionMm = d.value("linearDeflectionMm", 0.01);
			out.discretize.closedPreserveEndpoint = d.value("closedPreserveEndpoint", false);
			out.discretize.outputTangent = d.value("outputTangent", true);
			out.discretize.outputNormal = d.value("outputNormal", true);
		}
		return validateFeatureSpec(out, errMsg);
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

std::string featureSpecToJson(const FeatureSpec& spec)
{
	nlohmann::json j;
	j["schemaVersion"] = spec.schemaVersion;
	if (!spec.featureId.empty())
	{
		j["featureId"] = spec.featureId;
	}
	nlohmann::json wp;
	writeWorkpieceJson(wp, spec.workpiece);
	j["workpiece"] = wp;
	j["kind"] = featureKindToString(spec.kind);
	nlohmann::json refs;
	writeRefsJson(refs, spec.refs);
	j["refs"] = refs;
	nlohmann::json disc;
	disc["stepMm"] = spec.discretize.stepMm;
	disc["linearDeflectionMm"] = spec.discretize.linearDeflectionMm;
	disc["closedPreserveEndpoint"] = spec.discretize.closedPreserveEndpoint;
	disc["outputTangent"] = spec.discretize.outputTangent;
	disc["outputNormal"] = spec.discretize.outputNormal;
	j["discretize"] = disc;
	return j.dump(2);
}

std::string featureCatalogToJson(const FeatureCatalog& catalog)
{
	nlohmann::json j;
	j["stepPathUtf8"] = catalog.stepPathUtf8;
	j["backendIdUtf8"] = catalog.backendIdUtf8;
	nlohmann::json arr = nlohmann::json::array();
	for (const FeatureCandidate& c : catalog.candidates)
	{
		nlohmann::json item;
		item["candidateId"] = c.candidateId;
		item["suggestedKind"] = featureKindToString(c.suggestedKind);
		item["summary"] = c.summary;
		item["lengthMm"] = c.lengthMm;
		item["areaMm2"] = c.areaMm2;
		item["dihedralDeg"] = c.dihedralDeg;
		nlohmann::json refs;
		writeRefsJson(refs, c.refs);
		item["refs"] = refs;
		arr.push_back(item);
	}
	j["candidates"] = arr;
	return j.dump(2);
}

bool suggestFeaturesFromCatalog(
	const FeatureCatalog& catalog,
	const std::string& intentUtf8,
	std::vector<FeatureSpec>& out,
	std::string* errMsg)
{
	out.clear();
	const std::string lower = [&intentUtf8]() {
		std::string s = intentUtf8;
		for (char& c : s)
		{
			if (c >= 'A' && c <= 'Z')
			{
				c = static_cast<char>(c - 'A' + 'a');
			}
		}
		return s;
	}();
	const bool weld = lower.find("焊") != std::string::npos || lower.find("weld") != std::string::npos;
	const bool glue = lower.find("胶") != std::string::npos || lower.find("glue") != std::string::npos;
	const bool grind = lower.find("磨") != std::string::npos || lower.find("grind") != std::string::npos;

	for (const FeatureCandidate& c : catalog.candidates)
	{
		FeatureSpec spec;
		spec.workpiece.backendIdUtf8 = catalog.backendIdUtf8;
		spec.workpiece.stepPathUtf8 = catalog.stepPathUtf8;
		spec.featureId = c.candidateId;
		spec.refs = c.refs;

		if (weld && !c.refs.edgeIndices.empty() && c.dihedralDeg > 30.0 && c.dihedralDeg < 150.0)
		{
			spec.kind = FeatureKind::EdgeChain;
			spec.discretize.stepMm = 5.0;
			out.push_back(spec);
		}
		else if (glue && !c.refs.faceIndices.empty() && c.areaMm2 > 0.0 && c.areaMm2 < 50000.0)
		{
			spec.kind = FeatureKind::FaceBoundary;
			spec.discretize.stepMm = 2.0;
			out.push_back(spec);
		}
		else if (grind && !c.refs.faceIndices.empty() && c.areaMm2 > 10000.0)
		{
			spec.kind = FeatureKind::FaceUVGrid;
			spec.refs.uvCountU = 16;
			spec.refs.uvCountV = 16;
			spec.discretize.stepMm = 0.0;
			out.push_back(spec);
		}
	}
	if (out.empty())
	{
		if (errMsg)
		{
			*errMsg = "no matching features for intent";
		}
		return false;
	}
	return true;
}

namespace
{

bool edgeByIndex(const TopoDS_Shape& shape, int index, TopoDS_Edge& out)
{
	int idx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next(), ++idx)
	{
		if (idx == index)
		{
			out = TopoDS::Edge(exp.Current());
			return true;
		}
	}
	return false;
}

bool faceByIndex(const TopoDS_Shape& shape, int index, TopoDS_Face& out)
{
	int idx = 0;
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next(), ++idx)
	{
		if (idx == index)
		{
			out = TopoDS::Face(exp.Current());
			return true;
		}
	}
	return false;
}

void copyGpPnt(const gp_Pnt& p, double out[3])
{
	out[0] = p.X();
	out[1] = p.Y();
	out[2] = p.Z();
}

/// 标签沿工件外法向偏移，避免贴在棱线上与编号重叠
gp_Vec labelOutwardFromBbox(const TopoDS_Shape& shape, const gp_Pnt& anchor, const gp_Vec& fallback)
{
	gp_Pnt center;
	double diagonal = 50.0;
	Bnd_Box box;
	BRepBndLib::Add(shape, box);
	if (!box.IsVoid())
	{
		Standard_Real xmin = 0.0;
		Standard_Real ymin = 0.0;
		Standard_Real zmin = 0.0;
		Standard_Real xmax = 0.0;
		Standard_Real ymax = 0.0;
		Standard_Real zmax = 0.0;
		box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
		center = gp_Pnt((xmin + xmax) * 0.5, (ymin + ymax) * 0.5, (zmin + zmax) * 0.5);
		const double dx = xmax - xmin;
		const double dy = ymax - ymin;
		const double dz = zmax - zmin;
		diagonal = std::sqrt(dx * dx + dy * dy + dz * dz);
	}
	gp_Vec outward(anchor.X() - center.X(), anchor.Y() - center.Y(), anchor.Z() - center.Z());
	if (outward.SquareMagnitude() < 1e-12)
	{
		outward = fallback;
		if (outward.SquareMagnitude() < 1e-12)
		{
			outward = gp_Vec(0.0, 0.0, 1.0);
		}
	}
	outward.Normalize();
	const double dist = std::clamp(diagonal * 0.12, 18.0, 55.0);
	outward *= dist;
	return outward;
}

} // namespace

bool computeFeatureAnchor(const WorkpieceRef& workpiece, const FeatureRefs& refs, FeatureAnchor& out, std::string* errMsg)
{
	out = FeatureAnchor{};
	if (workpiece.stepPathUtf8.empty())
	{
		if (errMsg)
		{
			*errMsg = "stepPathUtf8 required";
		}
		return false;
	}
	TopoDS_Shape shape;
	if (!readStepShape(workpiece.stepPathUtf8, shape, errMsg))
	{
		return false;
	}

	if (!refs.edgeIndices.empty())
	{
		const int edgeIdx = refs.edgeIndices.front();
		TopoDS_Edge edge;
		if (!edgeByIndex(shape, edgeIdx, edge))
		{
			if (errMsg)
			{
				*errMsg = "edge index out of range";
			}
			return false;
		}
		BRepAdaptor_Curve curve(edge);
		const double u0 = curve.FirstParameter();
		const double u1 = curve.LastParameter();
		const double um = (u0 + u1) * 0.5;
		gp_Pnt p0;
		gp_Pnt p1;
		gp_Pnt pm;
		gp_Vec tan;
		curve.D0(u0, p0);
		curve.D0(u1, p1);
		curve.D1(um, pm, tan);
		copyGpPnt(pm, out.anchorXyzMm);
		out.hasEdgeSegment = true;
		copyGpPnt(p0, out.edgeEndAXyzMm);
		copyGpPnt(p1, out.edgeEndBXyzMm);
		const gp_Vec outward = labelOutwardFromBbox(shape, pm, tan);
		copyGpPnt(pm.Translated(outward), out.labelOffsetXyzMm);
		out.candidateId = "edge_" + std::to_string(edgeIdx);
		return true;
	}

	if (!refs.faceIndices.empty())
	{
		const int faceIdx = refs.faceIndices.front();
		TopoDS_Face face;
		if (!faceByIndex(shape, faceIdx, face))
		{
			if (errMsg)
			{
				*errMsg = "face index out of range";
			}
			return false;
		}
		GProp_GProps props;
		BRepGProp::SurfaceProperties(face, props);
		const gp_Pnt center = props.CentreOfMass();
		copyGpPnt(center, out.anchorXyzMm);
		BRepAdaptor_Surface surf(face);
		const double uMid = (surf.FirstUParameter() + surf.LastUParameter()) * 0.5;
		const double vMid = (surf.FirstVParameter() + surf.LastVParameter()) * 0.5;
		gp_Pnt ps;
		gp_Vec du;
		gp_Vec dv;
		surf.D1(uMid, vMid, ps, du, dv);
		gp_Vec normal = du.Crossed(dv);
		if (face.Orientation() == TopAbs_REVERSED)
		{
			normal.Reverse();
		}
		if (normal.Magnitude() > 1e-9)
		{
			normal.Normalize();
		}
		else
		{
			normal = gp_Vec(0.0, 0.0, 1.0);
		}
		const gp_Vec outward = labelOutwardFromBbox(shape, center, normal);
		copyGpPnt(center.Translated(outward), out.labelOffsetXyzMm);
		out.candidateId = "face_" + std::to_string(faceIdx);
		return true;
	}

	if (errMsg)
	{
		*errMsg = "FeatureRefs has no edge or face indices";
	}
	return false;
}

} // namespace geoalgo
