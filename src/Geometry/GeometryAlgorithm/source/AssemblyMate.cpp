/// @file AssemblyMate.cpp
/// @brief 装配一次定位：面几何 + 最小运动刚体增量

#include "AssemblyMate.h"

#include "ShapeQuery.h"
#include "detail/OccIncludes.h"

#include <BRepAdaptor_Surface.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <TopAbs.hxx>
#include <gp_Ax1.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>

#include <algorithm>
#include <cmath>
#include <string>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

namespace geoalgo
{
namespace
{
constexpr double kEps = 1e-9;

void setErr(std::string* err, const char* msg)
{
	if (err)
	{
		*err = msg;
	}
}

Eigen::Vector3d toV(const Point3d& p)
{
	return {p.x, p.y, p.z};
}

Point3d toP(const Eigen::Vector3d& v)
{
	return {v.x(), v.y(), v.z()};
}

Point3d toP(const gp_Pnt& p)
{
	return {p.X(), p.Y(), p.Z()};
}

Point3d toP(const gp_Dir& d)
{
	return {d.X(), d.Y(), d.Z()};
}

bool unitOrErr(Eigen::Vector3d& v, std::string* err, const char* msg)
{
	const double n = v.norm();
	if (n < kEps)
	{
		setErr(err, msg);
		return false;
	}
	v /= n;
	return true;
}

Eigen::Vector3d anyPerp(const Eigen::Vector3d& a)
{
	const Eigen::Vector3d cand = (std::abs(a.x()) < 0.9) ? Eigen::Vector3d::UnitX() : Eigen::Vector3d::UnitY();
	Eigen::Vector3d p = a.cross(cand);
	if (p.norm() < kEps)
	{
		p = a.cross(Eigen::Vector3d::UnitZ());
	}
	return p.normalized();
}

Eigen::Matrix3d rotFromTo(const Eigen::Vector3d& from, const Eigen::Vector3d& to)
{
	return Eigen::Quaterniond::FromTwoVectors(from, to).toRotationMatrix();
}

Eigen::Isometry3d rotAbout(const Eigen::Vector3d& pivot, const Eigen::Matrix3d& r)
{
	Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
	t.linear() = r;
	t.translation() = pivot - r * pivot;
	return t;
}

Eigen::Isometry3d translateBy(const Eigen::Vector3d& t)
{
	Eigen::Isometry3d d = Eigen::Isometry3d::Identity();
	d.translation() = t;
	return d;
}

Eigen::Vector3d alignedDir(const Eigen::Vector3d& grounded, AssemblyMateAlignment alignment)
{
	return alignment == AssemblyMateAlignment::Aligned ? grounded : -grounded;
}

double clampDot(const double d)
{
	return std::max(-1.0, std::min(1.0, d));
}

bool isRev(FaceMateSurfaceKind k)
{
	return k == FaceMateSurfaceKind::Cylinder || k == FaceMateSurfaceKind::Cone;
}

bool isSph(FaceMateSurfaceKind k)
{
	return k == FaceMateSurfaceKind::Sphere;
}

bool isPln(FaceMateSurfaceKind k)
{
	return k == FaceMateSurfaceKind::Plane;
}

Eigen::Vector3d mateDir(const FaceMateGeom& g)
{
	return toV(g.axisUnit);
}

Eigen::Vector3d matePivot(const FaceMateGeom& g)
{
	return toV(g.originMm);
}

/// 把 moving 方向转到与 grounded 成 targetRad（0=平行同向）
bool orientToAngle(const Eigen::Vector3d& gDir, const Eigen::Vector3d& mDir, const Eigen::Vector3d& pivot,
				   const double targetRad, Eigen::Isometry3d& out, std::string* err)
{
	Eigen::Vector3d g = gDir;
	Eigen::Vector3d m = mDir;
	if (!unitOrErr(g, err, "配合方向无效") || !unitOrErr(m, err, "配合方向无效"))
	{
		return false;
	}
	const double alpha = std::acos(clampDot(g.dot(m)));
	if (std::abs(alpha - targetRad) < 1e-8)
	{
		out = Eigen::Isometry3d::Identity();
		return true;
	}
	Eigen::Vector3d axis = g.cross(m);
	if (axis.norm() < kEps)
	{
		axis = anyPerp(g);
	}
	else
	{
		axis.normalize();
	}
	out = rotAbout(pivot, Eigen::AngleAxisd(targetRad - alpha, axis).toRotationMatrix());
	return true;
}

bool orientMatch(const Eigen::Vector3d& gDir, const Eigen::Vector3d& mDir, const Eigen::Vector3d& pivot,
				 const AssemblyMateAlignment alignment, Eigen::Isometry3d& out, std::string* err)
{
	Eigen::Vector3d g = alignedDir(gDir, alignment);
	Eigen::Vector3d m = mDir;
	if (!unitOrErr(g, err, "配合方向无效") || !unitOrErr(m, err, "配合方向无效"))
	{
		return false;
	}
	out = rotAbout(pivot, rotFromTo(m, g));
	return true;
}

bool orientPerp(const Eigen::Vector3d& gDir, const Eigen::Vector3d& mDir, const Eigen::Vector3d& pivot,
				Eigen::Isometry3d& out, std::string* err)
{
	return orientToAngle(gDir, mDir, pivot, M_PI * 0.5, out, err);
}

Eigen::Vector3d applyPt(const Eigen::Isometry3d& d, const Eigen::Vector3d& p)
{
	return d * p;
}

bool planePlaneDistanceAfterOrient(const FaceMateGeom& grounded, const FaceMateGeom& moving, const Eigen::Isometry3d& orient,
								   const double signedDistMm, Eigen::Isometry3d& out, std::string* err)
{
	Eigen::Vector3d n = toV(grounded.axisUnit);
	if (!unitOrErr(n, err, "平面法向无效"))
	{
		return false;
	}
	const Eigen::Vector3d pg = toV(grounded.originMm);
	const Eigen::Vector3d pm = applyPt(orient, toV(moving.originMm));
	const double cur = (pm - pg).dot(n);
	out = translateBy(n * (signedDistMm - cur)) * orient;
	return true;
}

bool concentricAxes(const FaceMateGeom& grounded, const FaceMateGeom& moving, const AssemblyMateAlignment alignment,
					Eigen::Isometry3d& out, std::string* err)
{
	Eigen::Isometry3d orient;
	if (!orientMatch(mateDir(grounded), mateDir(moving), matePivot(moving), alignment, orient, err))
	{
		return false;
	}
	Eigen::Vector3d ag = mateDir(grounded);
	if (!unitOrErr(ag, err, "轴线无效"))
	{
		return false;
	}
	const Eigen::Vector3d pg = matePivot(grounded);
	const Eigen::Vector3d pm = applyPt(orient, matePivot(moving));
	const Eigen::Vector3d d = pm - pg;
	const Eigen::Vector3d dPerp = d - d.dot(ag) * ag;
	out = translateBy(-dPerp) * orient;
	return true;
}

bool tangentPlaneRev(const FaceMateGeom& plane, const FaceMateGeom& rev, const bool movingIsRev, Eigen::Isometry3d& out,
					 std::string* err)
{
	// 轴∥平面后沿法向外切，侧向取当前轴在平面法向的符号
	Eigen::Vector3d n = mateDir(plane);
	Eigen::Vector3d ax = mateDir(rev);
	if (!unitOrErr(n, err, "平面法向无效") || !unitOrErr(ax, err, "轴线无效"))
	{
		return false;
	}
	const Eigen::Vector3d pivot = movingIsRev ? matePivot(rev) : matePivot(plane);
	Eigen::Isometry3d orient;
	if (movingIsRev)
	{
		if (!orientPerp(n, ax, pivot, orient, err))
		{
			return false;
		}
	}
	else if (!orientPerp(ax, n, pivot, orient, err))
	{
		return false;
	}
	Eigen::Vector3d nUse = movingIsRev ? n : (orient.linear() * n);
	if (!unitOrErr(nUse, err, "平面法向无效"))
	{
		return false;
	}
	const Eigen::Vector3d pPln = movingIsRev ? matePivot(plane) : applyPt(orient, matePivot(plane));
	const Eigen::Vector3d pAx = movingIsRev ? applyPt(orient, matePivot(rev)) : matePivot(rev);
	const double s = (pAx - pPln).dot(nUse);
	const double sign = (s >= 0.0) ? 1.0 : -1.0;
	const double target = sign * rev.radiusMm;
	const Eigen::Vector3d t = nUse * (target - s);
	out = translateBy(movingIsRev ? t : -t) * orient;
	return true;
}

} // namespace

bool queryFaceMateGeom(const ShapeHandle& shape, const int faceIndex, const Point3d* pickHint, FaceMateGeom& out,
					   std::string* errMsg)
{
	out = {};
	TopoDS_Shape native;
	if (!ShapeHandleAccess::nativeShape(shape, &native) || native.IsNull())
	{
		setErr(errMsg, "空形体");
		return false;
	}
	TopoDS_Face face;
	if (!shapeFaceAtIndex(native, faceIndex, face, errMsg))
	{
		return false;
	}

	BRepAdaptor_Surface surf(face, Standard_True);
	const bool reversed = face.Orientation() == TopAbs_REVERSED;
	const GeomAbs_SurfaceType ty = surf.GetType();

	switch (ty)
	{
	case GeomAbs_Plane:
	{
		gp_Dir n = surf.Plane().Position().Direction();
		if (reversed)
		{
			n.Reverse();
		}
		out.kind = FaceMateSurfaceKind::Plane;
		out.originMm = toP(surf.Plane().Position().Location());
		out.axisUnit = toP(n);
		break;
	}
	case GeomAbs_Cylinder:
	{
		const gp_Cylinder cyl = surf.Cylinder();
		out.kind = FaceMateSurfaceKind::Cylinder;
		out.originMm = toP(cyl.Location());
		out.axisUnit = toP(cyl.Axis().Direction());
		out.radiusMm = cyl.Radius();
		break;
	}
	case GeomAbs_Cone:
	{
		const gp_Cone cone = surf.Cone();
		out.kind = FaceMateSurfaceKind::Cone;
		out.originMm = toP(cone.Location());
		out.axisUnit = toP(cone.Axis().Direction());
		out.radiusMm = cone.RefRadius();
		out.radius2Mm = cone.SemiAngle();
		break;
	}
	case GeomAbs_Sphere:
	{
		const gp_Sphere sph = surf.Sphere();
		out.kind = FaceMateSurfaceKind::Sphere;
		out.originMm = toP(sph.Location());
		out.axisUnit = {0.0, 0.0, 1.0};
		out.radiusMm = sph.Radius();
		break;
	}
	case GeomAbs_Torus:
	{
		const gp_Torus tor = surf.Torus();
		out.kind = FaceMateSurfaceKind::Torus;
		out.originMm = toP(tor.Location());
		out.axisUnit = toP(tor.Axis().Direction());
		out.radiusMm = tor.MajorRadius();
		out.radius2Mm = tor.MinorRadius();
		break;
	}
	default:
		out.kind = FaceMateSurfaceKind::Other;
		out.originMm = toP(surf.Value(0.5 * (surf.FirstUParameter() + surf.LastUParameter()),
									  0.5 * (surf.FirstVParameter() + surf.LastVParameter())));
		out.axisUnit = {0.0, 0.0, 1.0};
		break;
	}

	if (pickHint)
	{
		out.pickHintMm = *pickHint;
	}
	else
	{
		out.pickHintMm = out.originMm;
	}
	return true;
}

bool computeAssemblyMateDelta(const FaceMateGeom& grounded, const FaceMateGeom& moving, const AssemblyMateParams& params,
							  Eigen::Isometry3d& outMovingDelta, std::string* errMsg)
{
	outMovingDelta = Eigen::Isometry3d::Identity();
	if (params.kind == AssemblyMateKind::Lock)
	{
		return true;
	}

	const FaceMateSurfaceKind gk = grounded.kind;
	const FaceMateSurfaceKind mk = moving.kind;
	const Eigen::Vector3d pivot = matePivot(moving);

	auto failCombo = [&](const char* msg) -> bool
	{
		setErr(errMsg, msg);
		return false;
	};

	switch (params.kind)
	{
	case AssemblyMateKind::Coincident:
		if (!isPln(gk) || !isPln(mk))
		{
			return failCombo("重合需要两个平面");
		}
		{
			Eigen::Isometry3d orient;
			if (!orientMatch(mateDir(grounded), mateDir(moving), pivot, params.alignment, orient, errMsg))
			{
				return false;
			}
			return planePlaneDistanceAfterOrient(grounded, moving, orient, 0.0, outMovingDelta, errMsg);
		}
	case AssemblyMateKind::Parallel:
		if (isPln(gk) && isPln(mk))
		{
			return orientMatch(mateDir(grounded), mateDir(moving), pivot, params.alignment, outMovingDelta, errMsg);
		}
		if (isPln(gk) && isRev(mk))
		{
			return orientPerp(mateDir(grounded), mateDir(moving), pivot, outMovingDelta, errMsg);
		}
		if (isRev(gk) && isPln(mk))
		{
			return orientPerp(mateDir(grounded), mateDir(moving), pivot, outMovingDelta, errMsg);
		}
		if (isRev(gk) && isRev(mk))
		{
			return orientMatch(mateDir(grounded), mateDir(moving), pivot, params.alignment, outMovingDelta, errMsg);
		}
		return failCombo("平行需要平面或圆柱/圆锥");
	case AssemblyMateKind::Perpendicular:
		if (isPln(gk) && isPln(mk))
		{
			return orientPerp(mateDir(grounded), mateDir(moving), pivot, outMovingDelta, errMsg);
		}
		if ((isPln(gk) && isRev(mk)) || (isRev(gk) && isPln(mk)))
		{
			return orientMatch(mateDir(grounded), mateDir(moving), pivot, AssemblyMateAlignment::Aligned,
							   outMovingDelta, errMsg);
		}
		if (isRev(gk) && isRev(mk))
		{
			return orientPerp(mateDir(grounded), mateDir(moving), pivot, outMovingDelta, errMsg);
		}
		return failCombo("垂直需要平面或圆柱/圆锥");
	case AssemblyMateKind::Distance:
		if (isPln(gk) && isPln(mk))
		{
			Eigen::Isometry3d orient;
			if (!orientMatch(mateDir(grounded), mateDir(moving), pivot, params.alignment, orient, errMsg))
			{
				return false;
			}
			const double signedDist =
				params.alignment == AssemblyMateAlignment::AntiAligned ? params.distanceMm : -params.distanceMm;
			return planePlaneDistanceAfterOrient(grounded, moving, orient, signedDist, outMovingDelta, errMsg);
		}
		if (isPln(gk) && isRev(mk))
		{
			Eigen::Isometry3d orient;
			if (!orientPerp(mateDir(grounded), mateDir(moving), pivot, orient, errMsg))
			{
				return false;
			}
			Eigen::Vector3d n = mateDir(grounded);
			if (!unitOrErr(n, errMsg, "平面法向无效"))
			{
				return false;
			}
			const Eigen::Vector3d pAx = applyPt(orient, matePivot(moving));
			const double s = (pAx - matePivot(grounded)).dot(n);
			const double sign = (params.alignment == AssemblyMateAlignment::AntiAligned) ? 1.0 : -1.0;
			outMovingDelta = translateBy(n * (sign * params.distanceMm - s)) * orient;
			return true;
		}
		if (isRev(gk) && isRev(mk))
		{
			Eigen::Isometry3d orient;
			if (!orientMatch(mateDir(grounded), mateDir(moving), pivot, params.alignment, orient, errMsg))
			{
				return false;
			}
			Eigen::Vector3d ag = mateDir(grounded);
			if (!unitOrErr(ag, errMsg, "轴线无效"))
			{
				return false;
			}
			const Eigen::Vector3d pm = applyPt(orient, matePivot(moving));
			const Eigen::Vector3d d = pm - matePivot(grounded);
			Eigen::Vector3d dPerp = d - d.dot(ag) * ag;
			if (dPerp.norm() < kEps)
			{
				dPerp = anyPerp(ag);
			}
			else
			{
				dPerp.normalize();
			}
			const Eigen::Vector3d target = dPerp * params.distanceMm;
			const Eigen::Vector3d cur = d - d.dot(ag) * ag;
			outMovingDelta = translateBy(target - cur) * orient;
			return true;
		}
		return failCombo("距离配合需要两平面、平面-圆柱或两圆柱");
	case AssemblyMateKind::Angle:
	{
		const double target = params.angleDeg * M_PI / 180.0;
		if ((isPln(gk) || isRev(gk)) && (isPln(mk) || isRev(mk)))
		{
			return orientToAngle(mateDir(grounded), mateDir(moving), pivot, target, outMovingDelta, errMsg);
		}
		return failCombo("角度配合需要平面或圆柱/圆锥");
	}
	case AssemblyMateKind::Tangent:
		if (isPln(gk) && isRev(mk))
		{
			return tangentPlaneRev(grounded, moving, true, outMovingDelta, errMsg);
		}
		if (isRev(gk) && isPln(mk))
		{
			return tangentPlaneRev(moving, grounded, false, outMovingDelta, errMsg);
		}
		if (isPln(gk) && isSph(mk))
		{
			Eigen::Vector3d n = mateDir(grounded);
			if (!unitOrErr(n, errMsg, "平面法向无效"))
			{
				return false;
			}
			const double s = (matePivot(moving) - matePivot(grounded)).dot(n);
			const double sign = (s >= 0.0) ? 1.0 : -1.0;
			outMovingDelta = translateBy(n * (sign * moving.radiusMm - s));
			return true;
		}
		if (isSph(gk) && isPln(mk))
		{
			Eigen::Vector3d n = mateDir(moving);
			if (!unitOrErr(n, errMsg, "平面法向无效"))
			{
				return false;
			}
			const double s = (matePivot(grounded) - matePivot(moving)).dot(n);
			const double sign = (s >= 0.0) ? 1.0 : -1.0;
			outMovingDelta = translateBy(n * (s - sign * grounded.radiusMm));
			return true;
		}
		if (isSph(gk) && isSph(mk))
		{
			Eigen::Vector3d d = matePivot(moving) - matePivot(grounded);
			if (d.norm() < kEps)
			{
				d = Eigen::Vector3d::UnitZ();
			}
			else
			{
				d.normalize();
			}
			const double target = grounded.radiusMm + moving.radiusMm;
			const Eigen::Vector3d cur = matePivot(moving) - matePivot(grounded);
			outMovingDelta = translateBy(d * target - cur);
			return true;
		}
		if (isRev(gk) && isRev(mk))
		{
			Eigen::Isometry3d orient;
			if (!orientMatch(mateDir(grounded), mateDir(moving), pivot, params.alignment, orient, errMsg))
			{
				return false;
			}
			Eigen::Vector3d ag = mateDir(grounded);
			if (!unitOrErr(ag, errMsg, "轴线无效"))
			{
				return false;
			}
			const Eigen::Vector3d pm = applyPt(orient, matePivot(moving));
			const Eigen::Vector3d d = pm - matePivot(grounded);
			Eigen::Vector3d dPerp = d - d.dot(ag) * ag;
			if (dPerp.norm() < kEps)
			{
				dPerp = anyPerp(ag);
			}
			else
			{
				dPerp.normalize();
			}
			const double target = grounded.radiusMm + moving.radiusMm;
			const Eigen::Vector3d cur = d - d.dot(ag) * ag;
			outMovingDelta = translateBy(dPerp * target - cur) * orient;
			return true;
		}
		if ((isSph(gk) && isRev(mk)) || (isRev(gk) && isSph(mk)))
		{
			const FaceMateGeom& sph = isSph(gk) ? grounded : moving;
			const FaceMateGeom& rev = isRev(gk) ? grounded : moving;
			const bool sphMoves = isSph(mk);
			Eigen::Vector3d ax = mateDir(rev);
			if (!unitOrErr(ax, errMsg, "轴线无效"))
			{
				return false;
			}
			const Eigen::Vector3d c = matePivot(sph);
			const Eigen::Vector3d p = matePivot(rev);
			Eigen::Vector3d dPerp = (c - p) - (c - p).dot(ax) * ax;
			if (dPerp.norm() < kEps)
			{
				dPerp = anyPerp(ax);
			}
			else
			{
				dPerp.normalize();
			}
			const double target = sph.radiusMm + rev.radiusMm;
			const Eigen::Vector3d cur = (c - p) - (c - p).dot(ax) * ax;
			const Eigen::Vector3d t = dPerp * target - cur;
			outMovingDelta = sphMoves ? translateBy(t) : translateBy(-t);
			return true;
		}
		return failCombo("相切需要平面与圆柱/球面，或圆柱/球面之间");
	case AssemblyMateKind::Concentric:
		if (isRev(gk) && isRev(mk))
		{
			return concentricAxes(grounded, moving, params.alignment, outMovingDelta, errMsg);
		}
		if (isSph(gk) && isSph(mk))
		{
			outMovingDelta = translateBy(matePivot(grounded) - matePivot(moving));
			return true;
		}
		if (isRev(gk) && isSph(mk))
		{
			Eigen::Vector3d ag = mateDir(grounded);
			if (!unitOrErr(ag, errMsg, "轴线无效"))
			{
				return false;
			}
			const Eigen::Vector3d d = matePivot(moving) - matePivot(grounded);
			outMovingDelta = translateBy(-(d - d.dot(ag) * ag));
			return true;
		}
		if (isSph(gk) && isRev(mk))
		{
			Eigen::Vector3d am = mateDir(moving);
			if (!unitOrErr(am, errMsg, "轴线无效"))
			{
				return false;
			}
			const Eigen::Vector3d d = matePivot(grounded) - matePivot(moving);
			outMovingDelta = translateBy(d - d.dot(am) * am);
			return true;
		}
		return failCombo("同轴心需要圆柱/圆锥轴线或球心");
	default:
		return failCombo("未知配合类型");
	}
}

} // namespace geoalgo
