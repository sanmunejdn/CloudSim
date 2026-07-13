#include "detail/ParamSurfaceDiscretize.h"

#include "FeatureDiscretizeParamUtils.h"
#include "detail/FeatureDiscretizeFrame.h"
#include "detail/OccIncludes.h"
#include "ShapeQuery.h"

#include <BRepClass_FaceClassifier.hxx>
#include <BRepGProp.hxx>
#include <Extrema_ExtPC.hxx>
#include <GProp_GProps.hxx>
#include <Precision.hxx>
#include <ShapeUpgrade_UnifySameDomain.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec2d.hxx>

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace geoalgo
{
namespace
{

constexpr double kPi = 3.14159265358979323846;
constexpr double kSeamTol = Precision::Confusion() * 100.0;
constexpr int kUvArcLengthSampleSegments = 64;

struct ScanPoint
{
	Point3d position;
	Vec3d tangent;
	Vec3d normal;
	bool hasTangent = false;
	bool hasNormal = false;
};

struct FaceGridScanFrame
{
	gp_Pnt2d startUV;
	gp_Vec2d dir1UV;
	gp_Vec2d dir2UV;
	int rowCount = 0;
	double fracStart = 0.0;
	double fracEnd = 1.0;
};

using FaceRowGrid = std::vector<std::vector<ScanPoint>>;
using RowFracList = std::vector<std::vector<double>>;

struct FracInterval
{
	double lo = 0.0;
	double hi = 0.0;
};

enum class SampleRejectReason
{
	None,
	Domain,
	Build,
	NotFinite
};

Point3d pointFromGp(const gp_Pnt& p)
{
	return Point3d{p.X(), p.Y(), p.Z()};
}

Vec3d vecFromGp(const gp_Vec& v)
{
	return Vec3d{v.X(), v.Y(), v.Z()};
}

double pointDist(const Point3d& a, const Point3d& b)
{
	const double dx = a.x - b.x;
	const double dy = a.y - b.y;
	const double dz = a.z - b.z;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double vecDot(const Vec3d& a, const Vec3d& b)
{
	return a.x * b.x + a.y * b.y + a.z * b.z;
}

double vecLenSq(const Vec3d& v)
{
	return v.x * v.x + v.y * v.y + v.z * v.z;
}

void normalizeVec3(Vec3d& v)
{
	const double len = std::sqrt(vecLenSq(v));
	if (len > 1e-12)
	{
		v.x /= len;
		v.y /= len;
		v.z /= len;
	}
}

bool isFinitePoint(const Point3d& p)
{
	return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z);
}

bool isFiniteScanPoint(const ScanPoint& sp)
{
	if (!isFinitePoint(sp.position))
	{
		return false;
	}
	if (sp.hasNormal)
	{
		return std::isfinite(sp.normal.x) && std::isfinite(sp.normal.y) && std::isfinite(sp.normal.z);
	}
	if (sp.hasTangent)
	{
		return std::isfinite(sp.tangent.x) && std::isfinite(sp.tangent.y) && std::isfinite(sp.tangent.z);
	}
	return true;
}

void CollectFacesFromShape(const TopoDS_Shape& shape, std::vector<TopoDS_Face>& outFaces)
{
	for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next())
	{
		outFaces.push_back(TopoDS::Face(ex.Current()));
	}
}

bool MergeSelectedFaces(const std::vector<TopoDS_Face>& faces, TopoDS_Shape& outShape, TopoDS_Face& outRefFace)
{
	if (faces.empty())
	{
		return false;
	}
	if (faces.size() == 1)
	{
		outRefFace = faces.front();
		outShape = outRefFace;
		return true;
	}

	const double fuzzy = Precision::Confusion() * 10.0;
	TopoDS_Shape fused = faces.front();
	for (size_t i = 1; i < faces.size(); ++i)
	{
		BRepAlgoAPI_Fuse fuseOp(fused, faces[i]);
		fuseOp.SetFuzzyValue(fuzzy);
		fuseOp.Build();
		if (!fuseOp.IsDone())
		{
			return false;
		}
		fused = fuseOp.Shape();
	}

	ShapeUpgrade_UnifySameDomain unify(fused, Standard_True, Standard_True, Standard_True);
	unify.Build();
	fused = unify.Shape();
	if (fused.IsNull())
	{
		return false;
	}

	TopoDS_Face largestFace;
	double maxArea = -1.0;
	for (TopExp_Explorer ex(fused, TopAbs_FACE); ex.More(); ex.Next())
	{
		const TopoDS_Face candidate = TopoDS::Face(ex.Current());
		GProp_GProps props;
		BRepGProp::SurfaceProperties(candidate, props);
		const double area = props.Mass();
		if (area > maxArea)
		{
			maxArea = area;
			largestFace = candidate;
		}
	}
	if (largestFace.IsNull())
	{
		return false;
	}

	outRefFace = largestFace;
	outShape = fused;
	return true;
}

Handle(Geom_Surface) UnwrapBasisSurface(const Handle(Geom_Surface)& surf)
{
	if (surf.IsNull())
	{
		return surf;
	}
	const Handle(Geom_RectangularTrimmedSurface) trimmed =
		Handle(Geom_RectangularTrimmedSurface)::DownCast(surf);
	if (!trimmed.IsNull())
	{
		return trimmed->BasisSurface();
	}
	return surf;
}

bool SharesUnderlyingSurface(const Handle(Geom_Surface)& a, const Handle(Geom_Surface)& b)
{
	if (a.IsNull() || b.IsNull())
	{
		return false;
	}
	if (a == b)
	{
		return true;
	}
	const Handle(Geom_Surface) basisA = UnwrapBasisSurface(a);
	const Handle(Geom_Surface) basisB = UnwrapBasisSurface(b);
	return !basisA.IsNull() && basisA == basisB;
}

bool AllFacesShareBasisSurface(const std::vector<TopoDS_Face>& faces, const TopoDS_Face& refFace)
{
	TopLoc_Location refLoc;
	const Handle(Geom_Surface) refSurf = BRep_Tool::Surface(refFace, refLoc);
	if (refSurf.IsNull())
	{
		return false;
	}
	for (const TopoDS_Face& face : faces)
	{
		TopLoc_Location faceLoc;
		const Handle(Geom_Surface) faceSurf = BRep_Tool::Surface(face, faceLoc);
		if (!SharesUnderlyingSurface(faceSurf, refSurf))
		{
			return false;
		}
	}
	return true;
}

void FoldPeriodicCoordIntoFaceInterval(
	Standard_Real& coord,
	double lo,
	double hi,
	bool isPeriodic,
	double period)
{
	if (!isPeriodic || !std::isfinite(period) || period <= Precision::Confusion())
	{
		return;
	}
	if (!std::isfinite(coord) || !std::isfinite(lo) || !std::isfinite(hi))
	{
		return;
	}
	if (lo > hi)
	{
		std::swap(lo, hi);
	}
	const double margin = Precision::Confusion();
	for (int i = 0; i < 64 && coord < lo - margin; ++i)
	{
		coord += period;
	}
	for (int i = 0; i < 64 && coord > hi + margin; ++i)
	{
		coord -= period;
	}
}

void FoldPeriodicUVIntoFaceBounds(const TopoDS_Face& face, Standard_Real& u, Standard_Real& v)
{
	if (face.IsNull())
	{
		return;
	}
	double u1 = 0.0;
	double u2 = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;
	BRepTools::UVBounds(face, u1, u2, v1, v2);

	BRepAdaptor_Surface adaptor(face, Standard_True);
	if (adaptor.IsUPeriodic())
	{
		const double period = adaptor.UPeriod();
		FoldPeriodicCoordIntoFaceInterval(u, u1, u2, true, period);
	}
	if (adaptor.IsVPeriodic())
	{
		const double period = adaptor.VPeriod();
		FoldPeriodicCoordIntoFaceInterval(v, v1, v2, true, period);
	}
}

bool ClassifyUVOnFace(const TopoDS_Face& face, Standard_Real u, Standard_Real v)
{
	if (face.IsNull() || !std::isfinite(u) || !std::isfinite(v))
	{
		return false;
	}

	Standard_Real uFold = u;
	Standard_Real vFold = v;
	FoldPeriodicUVIntoFaceBounds(face, uFold, vFold);

	const double tol = Precision::Confusion() * 10.0;
	BRepClass_FaceClassifier classifier;
	classifier.Perform(face, gp_Pnt2d(uFold, vFold), tol);
	const TopAbs_State state = classifier.State();
	return state == TopAbs_IN || state == TopAbs_ON;
}

void ExpandPeriodicUBoundsIfTiled(
	const TopoDS_Face& refFace,
	const std::vector<TopoDS_Face>& faces,
	double& uMin,
	double& uMax)
{
	// 多片域 UV 并集已跨 seam，禁止拉满整周期
	if (faces.size() > 1 || refFace.IsNull())
	{
		return;
	}
	BRepAdaptor_Surface adaptor(refFace, Standard_True);
	if (!adaptor.IsUPeriodic())
	{
		return;
	}
	const double period = adaptor.UPeriod();
	if (!std::isfinite(period) || period <= Precision::Confusion())
	{
		return;
	}

	double totalUSpan = 0.0;
	for (const TopoDS_Face& face : faces)
	{
		double u1 = 0.0;
		double u2 = 0.0;
		double v1 = 0.0;
		double v2 = 0.0;
		BRepTools::UVBounds(face, u1, u2, v1, v2);
		totalUSpan += (u2 - u1);
	}
	if (totalUSpan + Precision::Confusion() >= period * 0.95
		&& (uMax - uMin) + Precision::Confusion() < period * 0.95)
	{
		uMax = uMin + period;
	}
}

bool ComputeCombinedUVBounds(
	const TopoDS_Face& refFace,
	const Handle(Geom_Surface)& geomSurf,
	const std::vector<TopoDS_Face>& faces,
	double& uMin,
	double& uMax,
	double& vMin,
	double& vMax)
{
	if (faces.empty() || geomSurf.IsNull())
	{
		return false;
	}

	const bool refOnlyBounds = faces.size() > 1 && !AllFacesShareBasisSurface(faces, refFace);
	if (refOnlyBounds)
	{
		BRepTools::UVBounds(refFace, uMin, uMax, vMin, vMax);
		return uMax > uMin && vMax > vMin;
	}

	uMin = vMin = 1e100;
	uMax = vMax = -1e100;
	for (const TopoDS_Face& face : faces)
	{
		double u1 = 0.0;
		double u2 = 0.0;
		double v1 = 0.0;
		double v2 = 0.0;
		BRepTools::UVBounds(face, u1, u2, v1, v2);
		uMin = std::min(uMin, u1);
		uMax = std::max(uMax, u2);
		vMin = std::min(vMin, v1);
		vMax = std::max(vMax, v2);
	}
	if (!(uMax > uMin && vMax > vMin))
	{
		return false;
	}
	ExpandPeriodicUBoundsIfTiled(refFace, faces, uMin, uMax);
	return true;
}

bool SnapPointToDomainFaces(const std::vector<TopoDS_Face>& faces, gp_Pnt& ioPoint, gp_Vec& outNormal)
{
	const double onFaceTol = kSeamTol;
	double bestDist = 1e100;
	bool found = false;
	gp_Pnt bestPoint;
	gp_Vec bestNormal;

	for (const TopoDS_Face& face : faces)
	{
		TopLoc_Location faceLoc;
		const Handle(Geom_Surface) surf = BRep_Tool::Surface(face, faceLoc);
		if (surf.IsNull())
		{
			continue;
		}

		const gp_Trsf& toGlobal = faceLoc.Transformation();
		gp_Trsf toLocal = toGlobal;
		toLocal.Invert();

		gp_Pnt ptLocal = ioPoint;
		ptLocal.Transform(toLocal);

		GeomAPI_ProjectPointOnSurf proj(ptLocal, surf);
		if (!proj.IsDone() || proj.NbPoints() < 1)
		{
			continue;
		}

		Standard_Real u = 0.0;
		Standard_Real v = 0.0;
		proj.LowerDistanceParameters(u, v);
		const double dist = proj.LowerDistance();
		if (dist > onFaceTol)
		{
			continue;
		}

		BRepClass_FaceClassifier classifier;
		classifier.Perform(face, gp_Pnt2d(u, v), Precision::Confusion());
		const TopAbs_State state = classifier.State();
		if (state != TopAbs_IN && state != TopAbs_ON)
		{
			continue;
		}

		gp_Pnt pOnSurf;
		gp_Vec du;
		gp_Vec dv;
		surf->D1(u, v, pOnSurf, du, dv);
		gp_Vec nLocal = du.Crossed(dv);
		const double mag2 = nLocal.SquareMagnitude();
		if (mag2 <= Precision::SquareConfusion())
		{
			continue;
		}
		nLocal /= std::sqrt(mag2);

		if (face.Orientation() == TopAbs_REVERSED)
		{
			nLocal.Reverse();
		}

		pOnSurf.Transform(toGlobal);
		nLocal.Transform(toGlobal);
		if (dist < bestDist)
		{
			bestDist = dist;
			bestPoint = pOnSurf;
			bestNormal = nLocal;
			found = true;
		}
	}

	if (!found)
	{
		return false;
	}

	ioPoint = bestPoint;
	outNormal = bestNormal;
	if (outNormal.Magnitude() > Precision::Confusion())
	{
		outNormal.Normalize();
	}
	return true;
}

bool GetBestFaceNormalAtPoint(const std::vector<TopoDS_Face>& faces, const gp_Pnt& ptWorld, gp_Vec& outNormal)
{
	gp_Pnt pt = ptWorld;
	return SnapPointToDomainFaces(faces, pt, outNormal);
}

bool IsPointInDomainFacesUV(
	const std::vector<TopoDS_Face>& domainFaces,
	const TopoDS_Face& refFace,
	const Handle(Geom_Surface)& refGeomSurf,
	const TopLoc_Location& refFaceLoc,
	Standard_Real u,
	Standard_Real v)
{
	if (domainFaces.empty() || refGeomSurf.IsNull())
	{
		return false;
	}

	for (const TopoDS_Face& face : domainFaces)
	{
		TopLoc_Location faceLoc;
		const Handle(Geom_Surface) faceSurf = BRep_Tool::Surface(face, faceLoc);
		if (faceSurf.IsNull())
		{
			continue;
		}

		if (SharesUnderlyingSurface(faceSurf, refGeomSurf))
		{
			if (ClassifyUVOnFace(face, u, v))
			{
				return true;
			}
			continue;
		}

		gp_Pnt pLocal = refGeomSurf->Value(u, v);
		const gp_Trsf& refToGlobal = refFaceLoc.Transformation();
		gp_Pnt ptWorld = pLocal;
		if (!refFaceLoc.IsIdentity())
		{
			ptWorld.Transform(refToGlobal);
		}

		const gp_Trsf& toGlobal = faceLoc.Transformation();
		gp_Trsf toLocal = toGlobal;
		toLocal.Invert();
		ptWorld.Transform(toLocal);

		GeomAPI_ProjectPointOnSurf proj(ptWorld, faceSurf);
		if (!proj.IsDone() || proj.NbPoints() < 1)
		{
			continue;
		}

		Standard_Real u2 = 0.0;
		Standard_Real v2 = 0.0;
		proj.LowerDistanceParameters(u2, v2);
		if (proj.LowerDistance() > kSeamTol)
		{
			continue;
		}

		if (ClassifyUVOnFace(face, u2, v2))
		{
			return true;
		}
	}
	return false;
}

bool ParseTrackPercentages(double& trackStartPct, double& trackEndPct)
{
	if (!std::isfinite(trackStartPct) || !std::isfinite(trackEndPct))
	{
		trackStartPct = 0.0;
		trackEndPct = 100.0;
		return true;
	}
	trackStartPct = std::max(0.0, std::min(100.0, trackStartPct));
	trackEndPct = std::max(0.0, std::min(100.0, trackEndPct));
	if (trackStartPct >= trackEndPct - 1e-6)
	{
		trackStartPct = 0.0;
		trackEndPct = 100.0;
	}
	return true;
}

gp_Pnt2d InterpolateRowUV(const gp_Pnt2d& rowStart, const gp_Vec2d& dir1UV, double frac)
{
	return rowStart.Translated(dir1UV.Scaled(frac));
}

double NormalizeAlphaRad(double alphaRad)
{
	if (!std::isfinite(alphaRad))
	{
		return 0.0;
	}
	if (alphaRad > kPi)
	{
		return std::fmod(alphaRad + kPi, 2.0 * kPi) - kPi;
	}
	if (alphaRad < -kPi)
	{
		return std::fmod(alphaRad - kPi, 2.0 * kPi) + kPi;
	}
	return alphaRad;
}

void CalculateUVScanFrame(
	double uMin,
	double uMax,
	double vMin,
	double vMax,
	double alphaRad,
	gp_Pnt2d& startUV,
	gp_Vec2d& dir1UV,
	gp_Vec2d& dir2UV)
{
	const double alpha = NormalizeAlphaRad(alphaRad);
	const double uLength = uMax - uMin;
	const double vLength = vMax - vMin;

	const double absSinAlpha = std::abs(std::sin(alpha));
	const double absCosAlpha = std::abs(std::cos(alpha));

	const double uSinAlpha = uLength * absSinAlpha;
	const double uCosAlpha = uLength * absCosAlpha;
	const double ussAlpha = uSinAlpha * absSinAlpha;
	const double uscAlpha = uSinAlpha * absCosAlpha;

	const double vSinAlpha = vLength * absSinAlpha;
	const double vCosAlpha = vLength * absCosAlpha;
	const double vcsAlpha = vCosAlpha * absSinAlpha;
	const double vccAlpha = vCosAlpha * absCosAlpha;

	double length = 0.0;

	if (-(kPi / 2.0) <= alpha && alpha < 0.0)
	{
		startUV.SetCoord(uMin - vcsAlpha, vMax - vccAlpha);
		length = vSinAlpha + uCosAlpha;
		dir1UV.SetCoord(length * absCosAlpha, -length * absSinAlpha);
		length = vCosAlpha + uSinAlpha;
		dir2UV.SetCoord(length * absSinAlpha, length * absCosAlpha);
	}
	else if (0.0 <= alpha && alpha < (kPi / 2.0))
	{
		startUV.SetCoord(uMin + ussAlpha, vMin - uscAlpha);
		length = uCosAlpha + vSinAlpha;
		dir1UV.SetCoord(length * absCosAlpha, length * absSinAlpha);
		length = uSinAlpha + vCosAlpha;
		dir2UV.SetCoord(-length * absSinAlpha, length * absCosAlpha);
	}
	else if ((kPi / 2.0) <= alpha && alpha <= kPi)
	{
		startUV.SetCoord(uMax + vcsAlpha, vMin + vccAlpha);
		length = vSinAlpha + uCosAlpha;
		dir1UV.SetCoord(-length * absCosAlpha, length * absSinAlpha);
		length = vCosAlpha + uSinAlpha;
		dir2UV.SetCoord(-length * absSinAlpha, -length * absCosAlpha);
	}
	else
	{
		startUV.SetCoord(uMax - ussAlpha, vMax + uscAlpha);
		length = uCosAlpha + vSinAlpha;
		dir1UV.SetCoord(-length * absCosAlpha, -length * absSinAlpha);
		length = uSinAlpha + vCosAlpha;
		dir2UV.SetCoord(length * absSinAlpha, -length * absCosAlpha);
	}
}

gp_Pnt EvalSurfacePoint(
	const Handle(Geom_Surface)& geomSurf,
	const gp_Trsf& toGlobal,
	bool applyTransform,
	double u,
	double v)
{
	gp_Pnt p = geomSurf->Value(u, v);
	if (applyTransform)
	{
		p.Transform(toGlobal);
	}
	return p;
}

struct UvArcLengthProfile
{
	std::vector<double> fracs;
	std::vector<double> lengths;
	double totalLength = 0.0;
};

bool BuildUvArcLengthProfile(
	const Handle(Geom_Surface)& geomSurf,
	const gp_Trsf& toGlobal,
	bool applyTransform,
	const gp_Pnt2d& rowStart,
	const gp_Vec2d& dirUV,
	double fracStart,
	double fracEnd,
	int sampleSegments,
	UvArcLengthProfile& outProfile)
{
	outProfile = UvArcLengthProfile();
	if (geomSurf.IsNull() || dirUV.Magnitude() <= Precision::Confusion())
	{
		return false;
	}
	if (fracEnd <= fracStart + Precision::Confusion())
	{
		return false;
	}

	sampleSegments = std::max(1, sampleSegments);
	outProfile.fracs.reserve(static_cast<size_t>(sampleSegments) + 1);
	outProfile.lengths.reserve(static_cast<size_t>(sampleSegments) + 1);

	gp_Pnt prevPoint;
	bool hasPrev = false;
	const double span = fracEnd - fracStart;
	for (int i = 0; i <= sampleSegments; ++i)
	{
		const double frac = fracStart + span * static_cast<double>(i) / sampleSegments;
		const gp_Pnt2d uv = InterpolateRowUV(rowStart, dirUV, frac);
		const gp_Pnt point = EvalSurfacePoint(geomSurf, toGlobal, applyTransform, uv.X(), uv.Y());
		if (hasPrev)
		{
			const double chord = prevPoint.Distance(point);
			if (std::isfinite(chord))
			{
				outProfile.totalLength += chord;
			}
		}
		outProfile.fracs.push_back(frac);
		outProfile.lengths.push_back(outProfile.totalLength);
		prevPoint = point;
		hasPrev = true;
	}
	return outProfile.totalLength > Precision::Confusion();
}

double InterpolateFracAtArcLength(const UvArcLengthProfile& profile, double targetLength)
{
	if (profile.fracs.empty() || profile.lengths.empty())
	{
		return 0.0;
	}
	if (targetLength <= 0.0)
	{
		return profile.fracs.front();
	}
	if (targetLength >= profile.totalLength)
	{
		return profile.fracs.back();
	}

	const auto it = std::lower_bound(profile.lengths.begin(), profile.lengths.end(), targetLength);
	size_t idx = static_cast<size_t>(std::distance(profile.lengths.begin(), it));
	if (idx == 0)
	{
		return profile.fracs.front();
	}
	if (idx >= profile.lengths.size())
	{
		return profile.fracs.back();
	}

	const double len0 = profile.lengths[idx - 1];
	const double len1 = profile.lengths[idx];
	const double frac0 = profile.fracs[idx - 1];
	const double frac1 = profile.fracs[idx];
	if (len1 <= len0 + Precision::Confusion())
	{
		return frac1;
	}
	const double t = (targetLength - len0) / (len1 - len0);
	return frac0 + t * (frac1 - frac0);
}

int ComputeScanRowCount(
	const Handle(Geom_Surface)& geomSurf,
	const gp_Trsf& toGlobal,
	const gp_Pnt2d& startUV,
	const gp_Vec2d& dir2UV,
	double rowSpacing,
	bool applyTransform)
{
	UvArcLengthProfile profile;
	if (!BuildUvArcLengthProfile(geomSurf, toGlobal, applyTransform, startUV, dir2UV,
			0.0, 1.0, kUvArcLengthSampleSegments, profile))
	{
		return 2;
	}
	return std::max(2, static_cast<int>(std::ceil(profile.totalLength / rowSpacing)) + 1);
}

int ComputeUniformSampleCount3D(
	const Handle(Geom_Surface)& geomSurf,
	const gp_Trsf& toGlobal,
	bool applyTransform,
	const gp_Pnt2d& rowStart,
	const gp_Vec2d& dir1UV,
	double fracStart,
	double fracEnd,
	double colSpacing)
{
	if (geomSurf.IsNull() || colSpacing <= Precision::Confusion())
	{
		return 2;
	}
	if (fracEnd <= fracStart + Precision::Confusion())
	{
		return 2;
	}
	UvArcLengthProfile profile;
	if (!BuildUvArcLengthProfile(geomSurf, toGlobal, applyTransform, rowStart, dir1UV,
			fracStart, fracEnd, kUvArcLengthSampleSegments, profile))
	{
		return 2;
	}
	return std::max(2, static_cast<int>(std::ceil(profile.totalLength / colSpacing)) + 1);
}

double PointLineDeflection(const gp_Pnt& a, const gp_Pnt& b, const gp_Pnt& p)
{
	const gp_Vec ab(a, b);
	const double ab2 = ab.SquareMagnitude();
	if (ab2 <= Precision::SquareConfusion())
	{
		return p.Distance(a);
	}
	const double t = std::max(0.0, std::min(1.0, gp_Vec(a, p).Dot(ab) / ab2));
	const gp_Pnt proj = a.Translated(ab.Scaled(t));
	return p.Distance(proj);
}

bool DiscretizeRowUniformUV(
	const Handle(Geom_Surface)& geomSurf,
	const gp_Trsf& toGlobal,
	bool applyTransform,
	const gp_Pnt2d& rowStart,
	const gp_Vec2d& dir1UV,
	double fracStart,
	double fracEnd,
	double colSpacing,
	std::vector<std::pair<double, gp_Pnt>>& outSamples)
{
	if (geomSurf.IsNull() || dir1UV.Magnitude() <= Precision::Confusion())
	{
		return false;
	}
	if (fracEnd <= fracStart + Precision::Confusion())
	{
		return false;
	}
	UvArcLengthProfile profile;
	if (!BuildUvArcLengthProfile(geomSurf, toGlobal, applyTransform, rowStart, dir1UV,
			fracStart, fracEnd, kUvArcLengthSampleSegments, profile))
	{
		return false;
	}
	const int pointCount = std::max(2,
		static_cast<int>(std::ceil(profile.totalLength / colSpacing)) + 1);
	int segmentCount = pointCount - 1;
	if (segmentCount < 1)
	{
		segmentCount = 1;
	}
	outSamples.clear();
	outSamples.reserve(static_cast<size_t>(segmentCount) + 1);
	for (int i = 0; i <= segmentCount; ++i)
	{
		const double t01 = static_cast<double>(i) / segmentCount;
		const double frac = InterpolateFracAtArcLength(profile, profile.totalLength * t01);
		const gp_Pnt2d uv = InterpolateRowUV(rowStart, dir1UV, frac);
		outSamples.emplace_back(frac,
			EvalSurfacePoint(geomSurf, toGlobal, applyTransform, uv.X(), uv.Y()));
	}
	return outSamples.size() >= 2;
}

bool DiscretizeRowChordHeightUV(
	const Handle(Geom_Surface)& geomSurf,
	const gp_Trsf& toGlobal,
	bool applyTransform,
	const gp_Pnt2d& rowStart,
	const gp_Vec2d& dir1UV,
	double fracStart,
	double fracEnd,
	double chordHeight,
	std::vector<std::pair<double, gp_Pnt>>& outSamples)
{
	if (geomSurf.IsNull() || chordHeight <= 0.0 || dir1UV.Magnitude() <= Precision::Confusion())
	{
		return false;
	}
	if (fracEnd <= fracStart + Precision::Confusion())
	{
		return false;
	}

	outSamples.clear();
	const gp_Pnt2d uvLo = InterpolateRowUV(rowStart, dir1UV, fracStart);
	const gp_Pnt2d uvHi = InterpolateRowUV(rowStart, dir1UV, fracEnd);
	outSamples.emplace_back(fracStart,
		EvalSurfacePoint(geomSurf, toGlobal, applyTransform, uvLo.X(), uvLo.Y()));
	outSamples.emplace_back(fracEnd,
		EvalSurfacePoint(geomSurf, toGlobal, applyTransform, uvHi.X(), uvHi.Y()));

	bool subdivided = true;
	const int maxSubdiv = 4096;
	int subdivCount = 0;
	while (subdivided && subdivCount < maxSubdiv)
	{
		subdivided = false;
		++subdivCount;
		for (size_t i = 0; i + 1 < outSamples.size(); ++i)
		{
			const double f0 = outSamples[i].first;
			const double f1 = outSamples[i + 1].first;
			const double fm = 0.5 * (f0 + f1);
			const gp_Pnt2d uvMid = InterpolateRowUV(rowStart, dir1UV, fm);
			const gp_Pnt pm = EvalSurfacePoint(geomSurf, toGlobal, applyTransform, uvMid.X(), uvMid.Y());
			if (PointLineDeflection(outSamples[i].second, outSamples[i + 1].second, pm) > chordHeight)
			{
				outSamples.insert(outSamples.begin() + static_cast<std::ptrdiff_t>(i) + 1, {fm, pm});
				subdivided = true;
				break;
			}
		}
	}
	return outSamples.size() >= 2;
}

bool GetFaceNormalAtRefUV(
	const TopoDS_Face& face,
	const Handle(Geom_Surface)& refGeomSurf,
	const TopLoc_Location& refFaceLoc,
	Standard_Real u,
	Standard_Real v,
	gp_Vec& outNormal)
{
	TopLoc_Location faceLoc;
	const Handle(Geom_Surface) faceSurf = BRep_Tool::Surface(face, faceLoc);
	if (faceSurf.IsNull())
	{
		return false;
	}

	Standard_Real uEval = u;
	Standard_Real vEval = v;
	if (!SharesUnderlyingSurface(faceSurf, refGeomSurf))
	{
		gp_Pnt pLocal = refGeomSurf->Value(u, v);
		gp_Pnt ptWorld = pLocal;
		if (!refFaceLoc.IsIdentity())
		{
			ptWorld.Transform(refFaceLoc.Transformation());
		}
		gp_Trsf toLocal = faceLoc.Transformation();
		toLocal.Invert();
		ptWorld.Transform(toLocal);
		GeomAPI_ProjectPointOnSurf proj(ptWorld, faceSurf);
		if (!proj.IsDone() || proj.NbPoints() < 1)
		{
			return false;
		}
		proj.LowerDistanceParameters(uEval, vEval);
	}
	FoldPeriodicUVIntoFaceBounds(face, uEval, vEval);

	BRepAdaptor_Surface adaptor(face, Standard_True);
	gp_Pnt pOnSurf;
	gp_Vec du;
	gp_Vec dv;
	adaptor.D1(uEval, vEval, pOnSurf, du, dv);
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
	outNormal = nLocal;
	if (outNormal.Magnitude() > Precision::Confusion())
	{
		outNormal.Normalize();
	}
	return true;
}

bool IsPointOnMergedDomain(const std::vector<TopoDS_Face>& domainFaces, const gp_Pnt& ptWorld)
{
	gp_Vec dummy;
	return GetBestFaceNormalAtPoint(domainFaces, ptWorld, dummy);
}

bool computeScanTangentAtUv(
	const TopoDS_Face& refFace,
	Standard_Real u,
	Standard_Real v,
	const gp_Vec2d& travelDirUV,
	gp_Vec& outTangent)
{
	gp_Vec2d dir = travelDirUV;
	if (dir.Magnitude() <= Precision::Confusion())
	{
		return false;
	}
	dir.Normalize();
	BRepAdaptor_Surface adaptor(refFace);
	gp_Pnt ps;
	gp_Vec du;
	gp_Vec dv;
	adaptor.D1(u, v, ps, du, dv);
	gp_Vec tan = du.Multiplied(dir.X()).Added(dv.Multiplied(dir.Y()));
	if (tan.Magnitude() <= Precision::Confusion())
	{
		return false;
	}
	tan.Normalize();
	outTangent = tan;
	return true;
}

bool BuildScanPointFromSample(
	const std::vector<TopoDS_Face>& domainFaces,
	const TopoDS_Face& refFace,
	const Handle(Geom_Surface)& refGeomSurf,
	const TopLoc_Location& refFaceLoc,
	const gp_Pnt& p,
	Standard_Real u,
	Standard_Real v,
	const gp_Vec2d* travelDirUV,
	ScanPoint& out)
{
	gp_Vec nVec;
	bool gotNormal = GetBestFaceNormalAtPoint(domainFaces, p, nVec);
	if (!gotNormal)
	{
		for (const TopoDS_Face& face : domainFaces)
		{
			if (GetFaceNormalAtRefUV(face, refGeomSurf, refFaceLoc, u, v, nVec))
			{
				gotNormal = true;
				break;
			}
		}
	}
	if (!gotNormal)
	{
		return false;
	}

	out.position = pointFromGp(p);
	out.normal = vecFromGp(nVec);
	normalizeVec3(out.normal);
	out.hasNormal = true;

	if (travelDirUV != nullptr)
	{
		gp_Vec tan;
		if (computeScanTangentAtUv(refFace, u, v, *travelDirUV, tan))
		{
			out.tangent = vecFromGp(tan);
			normalizeVec3(out.tangent);
			out.hasTangent = true;
		}
	}
	return true;
}

bool TryAcceptParamSurfaceSample(
	const std::vector<TopoDS_Face>& domainFaces,
	const TopoDS_Face& refFace,
	const Handle(Geom_Surface)& refGeomSurf,
	const TopLoc_Location& refFaceLoc,
	const gp_Pnt& p,
	Standard_Real u,
	Standard_Real v,
	bool checkDomain,
	const gp_Vec2d* travelDirUV,
	ScanPoint& out,
	SampleRejectReason* rejectReason)
{
	if (checkDomain)
	{
		bool inDomain = IsPointInDomainFacesUV(domainFaces, refFace, refGeomSurf, refFaceLoc, u, v);
		if (!inDomain)
		{
			inDomain = IsPointOnMergedDomain(domainFaces, p);
		}
		if (!inDomain)
		{
			if (rejectReason)
			{
				*rejectReason = SampleRejectReason::Domain;
			}
			return false;
		}
	}
	if (!BuildScanPointFromSample(
			domainFaces,
			refFace,
			refGeomSurf,
			refFaceLoc,
			p,
			u,
			v,
			travelDirUV,
			out))
	{
		if (rejectReason)
		{
			*rejectReason = SampleRejectReason::Build;
		}
		return false;
	}
	if (!isFiniteScanPoint(out))
	{
		if (rejectReason)
		{
			*rejectReason = SampleRejectReason::NotFinite;
		}
		return false;
	}
	if (rejectReason)
	{
		*rejectReason = SampleRejectReason::None;
	}
	return true;
}

void CollectRowValidFracIntervals(
	const std::vector<TopoDS_Face>& domainFaces,
	const TopoDS_Face& refFace,
	const Handle(Geom_Surface)& refGeomSurf,
	const TopLoc_Location& refFaceLoc,
	const gp_Pnt2d& rowStart,
	const gp_Vec2d& dir1UV,
	double fracStart,
	double fracEnd,
	int probeCount,
	std::vector<FracInterval>& outIntervals)
{
	outIntervals.clear();
	if (probeCount < 8)
	{
		probeCount = 8;
	}
	const double span = fracEnd - fracStart;
	if (span <= Precision::Confusion())
	{
		return;
	}

	std::vector<bool> inside(static_cast<size_t>(probeCount) + 1, false);
	for (int i = 0; i <= probeCount; ++i)
	{
		const double frac = fracStart + (static_cast<double>(i) / probeCount) * span;
		const gp_Pnt2d uv = InterpolateRowUV(rowStart, dir1UV, frac);
		inside[static_cast<size_t>(i)] = IsPointInDomainFacesUV(domainFaces, refFace, refGeomSurf,
			refFaceLoc, uv.X(), uv.Y());
	}

	int i = 0;
	while (i <= probeCount)
	{
		while (i <= probeCount && !inside[static_cast<size_t>(i)])
		{
			++i;
		}
		if (i > probeCount)
		{
			break;
		}
		int j = i;
		while (j <= probeCount && inside[static_cast<size_t>(j)])
		{
			++j;
		}
		FracInterval iv;
		iv.lo = fracStart + (static_cast<double>(i) / probeCount) * span;
		iv.hi = fracStart + (static_cast<double>(j - 1) / probeCount) * span;
		if (j == probeCount + 1)
		{
			iv.hi = fracEnd;
		}
		if (iv.hi > iv.lo + Precision::Confusion())
		{
			outIntervals.push_back(iv);
		}
		i = j;
	}
}

bool appendPointDedup(RawPath& path, const ScanPoint& sp, double seamTol)
{
	if (!path.points.empty())
	{
		if (pointDist(sp.position, path.points.back().positionMm) <= seamTol)
		{
			return false;
		}
	}
	RawPathPoint rp;
	rp.positionMm = sp.position;
	if (sp.hasTangent)
	{
		rp.tangent = sp.tangent;
		rp.hasTangent = true;
	}
	if (sp.hasNormal)
	{
		rp.normal = sp.normal;
		rp.hasNormal = true;
	}
	path.points.push_back(rp);
	return true;
}

size_t AppendPathRowToRawPath(
	RawPath& path,
	const std::vector<ScanPoint>& row,
	bool reverse,
	double seamTol)
{
	size_t appended = 0;
	if (reverse)
	{
		for (auto it = row.rbegin(); it != row.rend(); ++it)
		{
			ScanPoint sp = *it;
			if (sp.hasTangent)
			{
				sp.tangent.x = -sp.tangent.x;
				sp.tangent.y = -sp.tangent.y;
				sp.tangent.z = -sp.tangent.z;
			}
			if (appendPointDedup(path, sp, seamTol))
			{
				++appended;
			}
		}
		return appended;
	}
	for (const ScanPoint& pt : row)
	{
		if (appendPointDedup(path, pt, seamTol))
		{
			++appended;
		}
	}
	return appended;
}

bool TrimStitchedRowByTrackPercent(
	std::vector<ScanPoint>& row,
	double trackStartPct,
	double trackEndPct)
{
	if (row.size() < 2 || (trackStartPct <= 0.0 && trackEndPct >= 100.0))
	{
		return true;
	}

	std::vector<double> cumLen(row.size(), 0.0);
	for (size_t i = 1; i < row.size(); ++i)
	{
		cumLen[i] = cumLen[i - 1] + pointDist(row[i].position, row[i - 1].position);
	}

	const double totalLen = cumLen.back();
	if (totalLen <= Precision::Confusion())
	{
		return row.size() >= 2;
	}

	const double sLo = totalLen * (trackStartPct / 100.0);
	const double sHi = totalLen * (trackEndPct / 100.0);
	if (sHi <= sLo + Precision::Confusion())
	{
		return row.size() >= 2;
	}

	size_t iStart = 0;
	while (iStart + 1 < row.size() && cumLen[iStart + 1] < sLo)
	{
		++iStart;
	}
	size_t iEnd = row.size() - 1;
	while (iEnd > 0 && cumLen[iEnd] > sHi)
	{
		--iEnd;
	}
	if (iStart >= iEnd)
	{
		iStart = 0;
		iEnd = row.size() - 1;
	}

	std::vector<ScanPoint> trimmed;
	trimmed.reserve(iEnd - iStart + 1);
	for (size_t i = iStart; i <= iEnd; ++i)
	{
		trimmed.push_back(row[i]);
	}
	row = std::move(trimmed);
	return row.size() >= 2;
}

size_t DiscretizeOnParamFace(
	const TopoDS_Face& refFace,
	const std::vector<TopoDS_Face>& domainFaces,
	bool useUniform,
	double colSpacing,
	double chordHeight,
	double rowSpacing,
	bool serpentine,
	double dirAlphaRad,
	double trackStartPct,
	double trackEndPct,
	bool& reverseRow,
	RawPath& path)
{
	double uMin = 0.0;
	double uMax = 0.0;
	double vMin = 0.0;
	double vMax = 0.0;
	TopLoc_Location faceLoc;
	const Handle(Geom_Surface) geomSurf = BRep_Tool::Surface(refFace, faceLoc);
	if (geomSurf.IsNull())
	{
		return 0;
	}
	if (!ComputeCombinedUVBounds(refFace, geomSurf, domainFaces, uMin, uMax, vMin, vMax))
	{
		return 0;
	}

	const double fracStart = trackStartPct / 100.0;
	const double fracEnd = trackEndPct / 100.0;
	const bool useMultiDomainRows = domainFaces.size() > 1;
	const double seamTol = kSeamTol;

	gp_Pnt2d startUV;
	gp_Vec2d dir1UV;
	gp_Vec2d dir2UV;
	CalculateUVScanFrame(uMin, uMax, vMin, vMax, dirAlphaRad, startUV, dir1UV, dir2UV);

	const gp_Trsf& surfToGlobal = faceLoc.Transformation();
	const bool applyTransform = !faceLoc.IsIdentity();
	const int rowCount = ComputeScanRowCount(geomSurf, surfToGlobal, startUV, dir2UV, rowSpacing, applyTransform);
	const gp_Vec2d deltaDir2 = (rowCount > 1)
		? gp_Vec2d(dir2UV.X() / (rowCount - 1), dir2UV.Y() / (rowCount - 1))
		: gp_Vec2d(0.0, 0.0);

	const int probeCount = std::max(32,
		ComputeUniformSampleCount3D(geomSurf, surfToGlobal, applyTransform, startUV, dir1UV,
			fracStart, fracEnd, colSpacing)
			* 4);
	size_t pointCount = 0;

	for (int row = 0; row < rowCount; ++row)
	{
		const gp_Pnt2d rowStart = startUV.Translated(deltaDir2.Scaled(static_cast<double>(row)));
		std::vector<ScanPoint> rowPoints;

		if (useMultiDomainRows)
		{
			std::vector<FracInterval> intervals;
			CollectRowValidFracIntervals(domainFaces, refFace, geomSurf, faceLoc,
				rowStart, dir1UV, fracStart, fracEnd, probeCount, intervals);
			if (intervals.empty())
			{
				continue;
			}
			// 多域行：按有效 UV 段离散，段间 seam 去重
			for (const FracInterval& iv : intervals)
			{
				std::vector<std::pair<double, gp_Pnt>> samples;
				const bool ok = useUniform
					? DiscretizeRowUniformUV(geomSurf, surfToGlobal, applyTransform,
						rowStart, dir1UV, iv.lo, iv.hi, colSpacing, samples)
					: DiscretizeRowChordHeightUV(geomSurf, surfToGlobal, applyTransform,
						rowStart, dir1UV, iv.lo, iv.hi, chordHeight, samples);
				if (!ok || samples.empty())
				{
					continue;
				}
				for (const auto& sample : samples)
				{
					const gp_Pnt2d uvPt = InterpolateRowUV(rowStart, dir1UV, sample.first);
					ScanPoint sp;
					if (!TryAcceptParamSurfaceSample(domainFaces, refFace, geomSurf, faceLoc,
							sample.second, uvPt.X(), uvPt.Y(), false, &dir1UV, sp, nullptr))
					{
						continue;
					}
					if (!rowPoints.empty()
						&& pointDist(sp.position, rowPoints.back().position) <= seamTol)
					{
						continue;
					}
					rowPoints.push_back(sp);
				}
			}
		}
		else
		{
			std::vector<std::pair<double, gp_Pnt>> samples;
			const bool ok = useUniform
				? DiscretizeRowUniformUV(geomSurf, surfToGlobal, applyTransform,
					rowStart, dir1UV, fracStart, fracEnd, colSpacing, samples)
				: DiscretizeRowChordHeightUV(geomSurf, surfToGlobal, applyTransform,
					rowStart, dir1UV, fracStart, fracEnd, chordHeight, samples);
			if (!ok || samples.empty())
			{
				continue;
			}
			rowPoints.reserve(samples.size());
			for (const auto& sample : samples)
			{
				const gp_Pnt2d uvPt = InterpolateRowUV(rowStart, dir1UV, sample.first);
				ScanPoint sp;
				if (!TryAcceptParamSurfaceSample(domainFaces, refFace, geomSurf, faceLoc,
						sample.second, uvPt.X(), uvPt.Y(), true, &dir1UV, sp, nullptr))
				{
					continue;
				}
				rowPoints.push_back(sp);
			}
		}

		if (rowPoints.size() < 2)
		{
			continue;
		}

		const bool doReverse = serpentine ? reverseRow : false;
		if (doReverse)
		{
			std::reverse(rowPoints.begin(), rowPoints.end());
		}

		for (const ScanPoint& sp : rowPoints)
		{
			if (appendPointDedup(path, sp, seamTol))
			{
				++pointCount;
			}
		}

		if (serpentine)
		{
			reverseRow = !reverseRow;
		}
	}

	return pointCount;
}

bool DiscretizeSingleFaceToGrid(
	const TopoDS_Face& face,
	bool useUniform,
	double colSpacing,
	double chordHeight,
	double rowSpacing,
	double dirAlphaRad,
	double trackStartPct,
	double trackEndPct,
	FaceRowGrid& outGrid,
	RowFracList& outFracs,
	FaceGridScanFrame& outFrame)
{
	outGrid.clear();
	outFracs.clear();

	const std::vector<TopoDS_Face> domain = {face};
	double uMin = 0.0;
	double uMax = 0.0;
	double vMin = 0.0;
	double vMax = 0.0;
	TopLoc_Location faceLoc;
	const Handle(Geom_Surface) geomSurf = BRep_Tool::Surface(face, faceLoc);
	if (geomSurf.IsNull())
	{
		return false;
	}
	if (!ComputeCombinedUVBounds(face, geomSurf, domain, uMin, uMax, vMin, vMax))
	{
		return false;
	}

	const double fracStart = trackStartPct / 100.0;
	const double fracEnd = trackEndPct / 100.0;

	gp_Pnt2d startUV;
	gp_Vec2d dir1UV;
	gp_Vec2d dir2UV;
	CalculateUVScanFrame(uMin, uMax, vMin, vMax, dirAlphaRad, startUV, dir1UV, dir2UV);

	const gp_Trsf& surfToGlobal = faceLoc.Transformation();
	const bool applyTransform = !faceLoc.IsIdentity();
	const int rowCount = ComputeScanRowCount(geomSurf, surfToGlobal, startUV, dir2UV, rowSpacing, applyTransform);
	if (rowCount < 2)
	{
		return false;
	}

	const gp_Vec2d deltaDir2 = (rowCount > 1)
		? gp_Vec2d(dir2UV.X() / (rowCount - 1), dir2UV.Y() / (rowCount - 1))
		: gp_Vec2d(0.0, 0.0);

	outGrid.assign(static_cast<size_t>(rowCount), {});
	outFracs.assign(static_cast<size_t>(rowCount), {});

	for (int row = 0; row < rowCount; ++row)
	{
		const gp_Pnt2d rowStart = startUV.Translated(deltaDir2.Scaled(static_cast<double>(row)));
		std::vector<std::pair<double, gp_Pnt>> samples;
		const bool ok = useUniform
			? DiscretizeRowUniformUV(geomSurf, surfToGlobal, applyTransform,
				rowStart, dir1UV, fracStart, fracEnd, colSpacing, samples)
			: DiscretizeRowChordHeightUV(geomSurf, surfToGlobal, applyTransform,
				rowStart, dir1UV, fracStart, fracEnd, chordHeight, samples);

		std::vector<ScanPoint> rowPoints;
		std::vector<double> rowFracs;
		if (ok && !samples.empty())
		{
			rowPoints.reserve(samples.size());
			rowFracs.reserve(samples.size());
			for (const auto& sample : samples)
			{
				const gp_Pnt2d uvPt = InterpolateRowUV(rowStart, dir1UV, sample.first);
				ScanPoint sp;
				if (!TryAcceptParamSurfaceSample(domain, face, geomSurf, faceLoc, sample.second,
						uvPt.X(), uvPt.Y(), true, &dir1UV, sp, nullptr))
				{
					continue;
				}
				rowPoints.push_back(sp);
				rowFracs.push_back(sample.first);
			}
		}
		if (rowPoints.size() >= 2)
		{
			outGrid[static_cast<size_t>(row)] = std::move(rowPoints);
			outFracs[static_cast<size_t>(row)] = std::move(rowFracs);
		}
	}

	outFrame.startUV = startUV;
	outFrame.dir1UV = dir1UV;
	outFrame.dir2UV = dir2UV;
	outFrame.rowCount = rowCount;
	outFrame.fracStart = fracStart;
	outFrame.fracEnd = fracEnd;
	for (const auto& row : outGrid)
	{
		if (row.size() >= 2)
		{
			return true;
		}
	}
	return false;
}

size_t MapStitchRowToFaceRow(size_t stitchRow, size_t stitchRowCount, size_t faceRowCount)
{
	if (faceRowCount == 0)
	{
		return 0;
	}
	if (stitchRowCount <= 1 || faceRowCount <= 1)
	{
		return 0;
	}
	const size_t srcR = static_cast<size_t>(std::lround(
		static_cast<double>(stitchRow) * static_cast<double>(faceRowCount - 1)
		/ static_cast<double>(stitchRowCount - 1)));
	return std::min(srcR, faceRowCount - 1);
}

const std::vector<ScanPoint>* FaceRowAtStitchIndex(
	const FaceRowGrid& grid,
	size_t stitchRow,
	size_t stitchRowCount)
{
	if (grid.empty() || stitchRowCount == 0)
	{
		return nullptr;
	}
	const size_t faceRow = MapStitchRowToFaceRow(stitchRow, stitchRowCount, grid.size());
	if (faceRow >= grid.size() || grid[faceRow].size() < 2)
	{
		return nullptr;
	}
	return &grid[faceRow];
}

bool OrientFaceRowToFirstFace(const std::vector<ScanPoint>& refRow, std::vector<ScanPoint>& faceRow)
{
	if (refRow.size() < 2 || faceRow.size() < 2)
	{
		return false;
	}
	Vec3d refTan{
		refRow.back().position.x - refRow.front().position.x,
		refRow.back().position.y - refRow.front().position.y,
		refRow.back().position.z - refRow.front().position.z};
	Vec3d faceTan{
		faceRow.back().position.x - faceRow.front().position.x,
		faceRow.back().position.y - faceRow.front().position.y,
		faceRow.back().position.z - faceRow.front().position.z};
	if (vecLenSq(refTan) < 1e-18 || vecLenSq(faceTan) < 1e-18)
	{
		return false;
	}
	if (vecDot(refTan, faceTan) < 0.0)
	{
		std::reverse(faceRow.begin(), faceRow.end());
		return true;
	}
	return false;
}

void OrientStitchedRowToFirstFace(const std::vector<ScanPoint>& refRow, std::vector<ScanPoint>& stitchedRow)
{
	if (refRow.size() < 2 || stitchedRow.size() < 2)
	{
		return;
	}
	Vec3d refTan{
		refRow.back().position.x - refRow.front().position.x,
		refRow.back().position.y - refRow.front().position.y,
		refRow.back().position.z - refRow.front().position.z};
	if (vecLenSq(refTan) < 1e-18)
	{
		return;
	}
	const size_t refLen = std::min(refRow.size(), stitchedRow.size());
	Vec3d headTan{
		stitchedRow[refLen - 1].position.x - stitchedRow[0].position.x,
		stitchedRow[refLen - 1].position.y - stitchedRow[0].position.y,
		stitchedRow[refLen - 1].position.z - stitchedRow[0].position.z};
	if (vecLenSq(headTan) < 1e-18)
	{
		return;
	}
	if (vecDot(refTan, headTan) < 0.0)
	{
		std::reverse(stitchedRow.begin(), stitchedRow.end());
	}
}

std::vector<std::vector<ScanPoint>> AssembleHeteroRows(
	const std::vector<FaceRowGrid>& grids,
	size_t stitchRowCount,
	bool serpentine,
	bool& reverseRow,
	double trackStartPct,
	double trackEndPct)
{
	std::vector<std::vector<ScanPoint>> assembled;
	if (grids.empty())
	{
		return assembled;
	}

	const size_t rowCount = (stitchRowCount > 0) ? stitchRowCount : grids.front().size();
	assembled.reserve(rowCount);

	for (size_t r = 0; r < rowCount; ++r)
	{
		std::vector<ScanPoint> row;
		const std::vector<ScanPoint>* refRow = grids.empty()
			? nullptr
			: FaceRowAtStitchIndex(grids[0], r, rowCount);

		for (size_t f = 0; f < grids.size(); ++f)
		{
			const std::vector<ScanPoint>* faceRowPtr = FaceRowAtStitchIndex(grids[f], r, rowCount);
			if (!faceRowPtr)
			{
				continue;
			}
			std::vector<ScanPoint> faceRow = *faceRowPtr;

			if (f > 0 && refRow)
			{
				(void)OrientFaceRowToFirstFace(*refRow, faceRow);
			}

			row.insert(row.end(), faceRow.begin(), faceRow.end());
		}

		if (row.size() < 2)
		{
			continue;
		}

		if (refRow)
		{
			OrientStitchedRowToFirstFace(*refRow, row);
		}

		if (!TrimStitchedRowByTrackPercent(row, trackStartPct, trackEndPct))
		{
			continue;
		}

		if (serpentine && reverseRow)
		{
			std::reverse(row.begin(), row.end());
		}
		if (serpentine)
		{
			reverseRow = !reverseRow;
		}

		assembled.push_back(std::move(row));
	}
	return assembled;
}

size_t StitchHeteroRowGrids(
	const std::vector<FaceRowGrid>& grids,
	size_t stitchRowCount,
	bool serpentine,
	bool& reverseRow,
	double trackStartPct,
	double trackEndPct,
	RawPath& path)
{
	const double seamTol = kSeamTol;
	const auto assembled = AssembleHeteroRows(grids, stitchRowCount, serpentine, reverseRow,
		trackStartPct, trackEndPct);

	size_t pointCount = 0;
	for (const auto& row : assembled)
	{
		for (const ScanPoint& sp : row)
		{
			if (appendPointDedup(path, sp, seamTol))
			{
				++pointCount;
			}
		}
	}
	return pointCount;
}

size_t DiscretizeHeteroByRowStitch(
	const std::vector<TopoDS_Face>& orderedFaces,
	bool useUniform,
	double colSpacing,
	double chordHeight,
	double rowSpacing,
	bool serpentine,
	double dirAlphaRad,
	double trackStartPct,
	double trackEndPct,
	bool& reverseRow,
	RawPath& path)
{
	if (orderedFaces.empty())
	{
		return 0;
	}

	std::vector<FaceRowGrid> allGrids;
	allGrids.reserve(orderedFaces.size());
	size_t stitchRowCount = 0;
	const double perFaceTrackStartPct = 0.0;
	const double perFaceTrackEndPct = 100.0;

	for (const TopoDS_Face& face : orderedFaces)
	{
		FaceRowGrid grid;
		RowFracList fracs;
		FaceGridScanFrame frame;
		if (!DiscretizeSingleFaceToGrid(face, useUniform, colSpacing, chordHeight, rowSpacing,
				dirAlphaRad, perFaceTrackStartPct, perFaceTrackEndPct, grid, fracs, frame))
		{
			return 0;
		}
		stitchRowCount = std::max(stitchRowCount, static_cast<size_t>(frame.rowCount));
		allGrids.push_back(std::move(grid));
	}

	if (stitchRowCount < 2)
	{
		return 0;
	}

	return StitchHeteroRowGrids(allGrids, stitchRowCount, serpentine, reverseRow,
		trackStartPct, trackEndPct, path);
}

double SharedEdgeTolerance()
{
	return std::max(Precision::Confusion() * 10000.0, 1e-4);
}

bool IsPointOnEdge(const gp_Pnt& point, const TopoDS_Edge& edge, double tol)
{
	if (edge.IsNull())
	{
		return false;
	}
	BRepAdaptor_Curve adaptor(edge);
	const Standard_Real first = adaptor.FirstParameter();
	const Standard_Real last = adaptor.LastParameter();
	if (!std::isfinite(first) || !std::isfinite(last) || last <= first)
	{
		return false;
	}

	const double tolSq = tol * tol;
	double bestSqDist = 1e100;
	Extrema_ExtPC extrema(point, adaptor, first, last, Precision::Confusion());
	if (extrema.IsDone())
	{
		for (Standard_Integer i = 1; i <= extrema.NbExt(); ++i)
		{
			bestSqDist = std::min(bestSqDist, static_cast<double>(extrema.SquareDistance(i)));
		}
		Standard_Real distFirst = 0.0;
		Standard_Real distLast = 0.0;
		gp_Pnt pFirst;
		gp_Pnt pLast;
		extrema.TrimmedSquareDistances(distFirst, distLast, pFirst, pLast);
		bestSqDist = std::min(bestSqDist, static_cast<double>(distFirst));
		bestSqDist = std::min(bestSqDist, static_cast<double>(distLast));
	}
	if (!std::isfinite(bestSqDist) || bestSqDist >= 1e99)
	{
		const gp_Pnt pFirst = adaptor.Value(first);
		const gp_Pnt pLast = adaptor.Value(last);
		bestSqDist = std::min(point.SquareDistance(pFirst), point.SquareDistance(pLast));
	}
	return bestSqDist <= tolSq;
}

bool IsEverySampleOnEdge(const TopoDS_Edge& source, const TopoDS_Edge& target, double tol)
{
	if (source.IsNull() || target.IsNull())
	{
		return false;
	}
	BRepAdaptor_Curve curve(source);
	const Standard_Real first = curve.FirstParameter();
	const Standard_Real last = curve.LastParameter();
	if (!std::isfinite(first) || !std::isfinite(last) || last <= first)
	{
		return false;
	}

	const int sampleCount = 8;
	for (int i = 0; i <= sampleCount; ++i)
	{
		const Standard_Real t = first + (last - first)
			* static_cast<Standard_Real>(i) / static_cast<Standard_Real>(sampleCount);
		const gp_Pnt p = curve.Value(t);
		if (!IsPointOnEdge(p, target, tol))
		{
			return false;
		}
	}
	return true;
}

bool EdgesGeometricallyCoincident(const TopoDS_Edge& a, const TopoDS_Edge& b, double tol)
{
	if (a.IsNull() || b.IsNull())
	{
		return false;
	}
	return IsEverySampleOnEdge(a, b, tol) && IsEverySampleOnEdge(b, a, tol);
}

bool EdgesMatch(const TopoDS_Edge& a, const TopoDS_Edge& b, double tol)
{
	if (a.IsNull() || b.IsNull())
	{
		return false;
	}
	return a.IsSame(b) || EdgesGeometricallyCoincident(a, b, tol);
}

void CollectSharedEdgesWithFaces(
	const TopoDS_Face& face,
	const std::vector<TopoDS_Face>& otherFaces,
	double edgeTol,
	std::vector<TopoDS_Edge>& outEdges)
{
	outEdges.clear();
	for (TopExp_Explorer exFace(face, TopAbs_EDGE); exFace.More(); exFace.Next())
	{
		const TopoDS_Edge edgeOnFace = TopoDS::Edge(exFace.Current());
		bool shared = false;
		for (const TopoDS_Face& otherFace : otherFaces)
		{
			for (TopExp_Explorer exOther(otherFace, TopAbs_EDGE); exOther.More(); exOther.Next())
			{
				const TopoDS_Edge otherEdge = TopoDS::Edge(exOther.Current());
				if (EdgesMatch(edgeOnFace, otherEdge, edgeTol))
				{
					shared = true;
					break;
				}
			}
			if (shared)
			{
				break;
			}
		}
		if (!shared)
		{
			continue;
		}

		const bool alreadyAdded = std::any_of(outEdges.begin(), outEdges.end(),
			[&](const TopoDS_Edge& e) { return EdgesMatch(e, edgeOnFace, edgeTol); });
		if (!alreadyAdded)
		{
			outEdges.push_back(edgeOnFace);
		}
	}
}

bool IsPointOnAnyEdge(const Point3d& pos, const std::vector<TopoDS_Edge>& edges, double tol)
{
	if (edges.empty())
	{
		return false;
	}
	const gp_Pnt point(pos.x, pos.y, pos.z);
	for (const TopoDS_Edge& edge : edges)
	{
		if (IsPointOnEdge(point, edge, tol))
		{
			return true;
		}
	}
	return false;
}

bool IsPathRowOnAnyEdge(const std::vector<ScanPoint>& row, const std::vector<TopoDS_Edge>& edges, double edgeTol)
{
	if (row.size() < 2 || edges.empty())
	{
		return false;
	}
	size_t onEdgeCount = 0;
	for (const ScanPoint& pt : row)
	{
		if (IsPointOnAnyEdge(pt.position, edges, edgeTol))
		{
			++onEdgeCount;
		}
	}
	const size_t allowedMisses = (row.size() <= 3) ? 0 : 1;
	return onEdgeCount + allowedMisses >= row.size();
}

size_t DiscretizePerFaceMode(
	const std::vector<TopoDS_Face>& inputFaces,
	bool useUniform,
	double colSpacing,
	double chordHeight,
	double rowSpacing,
	bool serpentine,
	double dirAlphaRad,
	double trackStartPct,
	double trackEndPct,
	bool& reverseRow,
	RawPath& path)
{
	std::vector<TopoDS_Face> processedFaces;
	processedFaces.reserve(inputFaces.size());
	size_t appendedPerFacePoints = 0;
	const double internalEdgeTol = std::min(
		std::max(SharedEdgeTolerance(), rowSpacing * 0.01), 0.05);
	const double seamTol = kSeamTol;

	for (const TopoDS_Face& face : inputFaces)
	{
		FaceRowGrid grid;
		RowFracList fracs;
		FaceGridScanFrame frame;
		if (!DiscretizeSingleFaceToGrid(face, useUniform, colSpacing, chordHeight, rowSpacing,
				dirAlphaRad, trackStartPct, trackEndPct, grid, fracs, frame))
		{
			continue;
		}

		std::vector<TopoDS_Edge> internalEdges;
		CollectSharedEdgesWithFaces(face, processedFaces, internalEdgeTol, internalEdges);

		size_t faceRowsWritten = 0;
		for (const auto& row : grid)
		{
			if (row.size() < 2)
			{
				continue;
			}
			if (IsPathRowOnAnyEdge(row, internalEdges, internalEdgeTol))
			{
				continue;
			}

			const bool doReverse = serpentine ? reverseRow : false;
			const size_t appended = AppendPathRowToRawPath(path, row, doReverse, seamTol);
			if (appended == 0)
			{
				continue;
			}
			appendedPerFacePoints += appended;
			++faceRowsWritten;
			if (serpentine)
			{
				reverseRow = !reverseRow;
			}
		}

		if (faceRowsWritten > 0)
		{
			processedFaces.push_back(face);
		}
	}

	return appendedPerFacePoints;
}

void normalizeDirAlphaDeg(double& dirAlphaDeg)
{
	if (!std::isfinite(dirAlphaDeg))
	{
		dirAlphaDeg = 0.0;
	}
	while (dirAlphaDeg > 180.0)
	{
		dirAlphaDeg -= 360.0;
	}
	while (dirAlphaDeg <= -180.0)
	{
		dirAlphaDeg += 360.0;
	}
}

bool collectInputFaces(
	const TopoDS_Shape& shape,
	const FeatureGeometry& geometry,
	std::vector<TopoDS_Face>& outFaces,
	std::string* errMsg)
{
	outFaces.clear();
	if (geometry.faceIndices.empty())
	{
		if (errMsg)
		{
			*errMsg = "faceIndices is empty";
		}
		return false;
	}
	for (int idx : geometry.faceIndices)
	{
		TopoDS_Face face;
		std::string faceErr;
		if (!shapeFaceAtIndex(shape, idx, face, &faceErr))
		{
			if (errMsg)
			{
				*errMsg = faceErr.empty() ? "invalid face index" : faceErr;
			}
			return false;
		}
		outFaces.push_back(face);
	}
	return !outFaces.empty();
}

void finalizeParamSurfacePath(const FeatureDiscretizeInput& input, RawPath& out)
{
	const DiscretizeParams disc = buildDiscretizeParams(input.params);
	detail::assignPathChordTangents(out, false, disc.outputTangent, &out.segmentEndExclusive, true);
}

} // namespace

bool discretizeFaceParamSurface(
	const TopoDS_Shape& shape,
	const FeatureDiscretizeInput& input,
	RawPath& out,
	std::string* errMsg)
{
	out.points.clear();

	const double rowSpacing = paramDouble(input.params, "stepMm", 2.0);
	const double colSpacing = paramDouble(input.params, "colSpacingMm", 1.0);
	const double chordHeight = paramDouble(input.params, "linearDeflectionMm", 0.01);
	double dirAlphaDeg = paramDouble(input.params, "gridAngleDeg", 0.0);
	normalizeDirAlphaDeg(dirAlphaDeg);
	const double dirAlphaRad = dirAlphaDeg * kPi / 180.0;

	double trackStartPct = paramDouble(input.params, "trackStartPct", 0.0);
	double trackEndPct = paramDouble(input.params, "trackEndPct", 100.0);
	ParseTrackPercentages(trackStartPct, trackEndPct);

	const std::string heteroMode = paramString(input.params, "heteroCombineMode", "Auto");
	const bool heteroRowStitch = heteroMode == "RowStitch";
	const bool forcePerFaceDiscretize = heteroMode == "PerFace";
	const bool unifySameBasis = paramBool(input.params, "unifySameBasis", true);

	const std::string edgeMode = paramString(input.params, "edgeDiscretizeMode", "Uniform");
	const bool useUniform = edgeMode == "Uniform";
	const bool useDeflection = edgeMode == "ChordHeight";
	if (!useUniform && !useDeflection)
	{
		if (errMsg)
		{
			*errMsg = "unknown edgeDiscretizeMode";
		}
		return false;
	}
	if (rowSpacing < 0.1)
	{
		if (errMsg)
		{
			*errMsg = "rowSpacing too small";
		}
		return false;
	}
	if (useUniform && colSpacing < 0.1)
	{
		if (errMsg)
		{
			*errMsg = "colSpacing too small";
		}
		return false;
	}

	const bool serpentine = paramString(input.params, "trajConnectMode", "Bow") == "Bow";

	std::vector<TopoDS_Face> inputFaces;
	if (!collectInputFaces(shape, input.geometry, inputFaces, errMsg))
	{
		return false;
	}

	const std::size_t pathBefore = out.points.size();
	bool reverseRow = paramBool(input.params, "reverseLayer", false);

	if (heteroRowStitch && inputFaces.size() > 1)
	{
		const size_t stitched = DiscretizeHeteroByRowStitch(inputFaces, useUniform, colSpacing,
			chordHeight, rowSpacing, serpentine, dirAlphaRad, trackStartPct, trackEndPct,
			reverseRow, out);
		if (stitched > 0 && out.points.size() > pathBefore)
		{
			finalizeParamSurfacePath(input, out);
			return true;
		}
	}

	if (forcePerFaceDiscretize && inputFaces.size() > 1)
	{
		const size_t perFacePts = DiscretizePerFaceMode(inputFaces, useUniform, colSpacing,
			chordHeight, rowSpacing, serpentine, dirAlphaRad, trackStartPct, trackEndPct,
			reverseRow, out);
		if (perFacePts > 0 && out.points.size() > pathBefore)
		{
			finalizeParamSurfacePath(input, out);
			return true;
		}
		if (errMsg)
		{
			*errMsg = "per-face discretization produced no points";
		}
		return false;
	}

	TopoDS_Shape mergedShape;
	TopoDS_Face refFace;
	const bool merged = MergeSelectedFaces(inputFaces, mergedShape, refFace);

	auto runDiscretize = [&](const TopoDS_Face& ref, const std::vector<TopoDS_Face>& domain) {
		(void)DiscretizeOnParamFace(ref, domain, useUniform, colSpacing, chordHeight, rowSpacing,
			serpentine, dirAlphaRad, trackStartPct, trackEndPct, reverseRow, out);
	};

	if (merged)
	{
		std::vector<TopoDS_Face> domainFaces;
		CollectFacesFromShape(mergedShape, domainFaces);
		if (domainFaces.empty())
		{
			domainFaces.push_back(refFace);
		}

		if (domainFaces.size() == 1)
		{
			runDiscretize(refFace, domainFaces);
		}
		else if (unifySameBasis && AllFacesShareBasisSurface(domainFaces, refFace))
		{
			runDiscretize(refFace, domainFaces);
		}
		else
		{
			for (const TopoDS_Face& face : inputFaces)
			{
				runDiscretize(face, {face});
			}
		}
	}
	else
	{
		for (const TopoDS_Face& face : inputFaces)
		{
			runDiscretize(face, {face});
		}
	}

	if (out.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "param surface discretization produced no points";
		}
		return false;
	}
	finalizeParamSurfacePath(input, out);
	return true;
}

} // namespace geoalgo
