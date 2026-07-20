/// @file TemplateBrepUpdate.cpp
/// @brief 获取面的 OCCT 曲面类型名称（用于调试输出）

#include "TemplateBrepUpdate.h"

#include "RunLogger.h"
#include "ShapeHandle.h"
#include "detail/OccIncludes.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <sstream>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <Geom_ConicalSurface.hxx>
#include <Geom_SphericalSurface.hxx>
#include <Geom_ToroidalSurface.hxx>
#include <gp_Cone.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace geoalgo
{
namespace
{
static const double kPi = 3.14159265358979323846;

using Vec3 = Eigen::Vector3d;

struct PoleDeltaAccum
{
	Vec3 delta = Vec3::Zero();
	double weightSum = 0.0;
};

struct OutlierSample
{
	double u = 0.0;
	double v = 0.0;
	gp_Pnt localTarget;
};

double clamp01(const double v)
{
	return std::max(0.0, std::min(1.0, v));
}

double normalAngleDeg(const Vec3& a, const Vec3& b)
{
	const double na = a.norm();
	const double nb = b.norm();
	if (na < 1e-9 || nb < 1e-9)
	{
		return 0.0;
	}
	const double dot = std::abs(a.dot(b) / (na * nb));
	return std::acos(clamp01(dot)) * 180.0 / kPi;
}

bool collectFaces(const TopoDS_Shape& shape, std::vector<TopoDS_Face>& outFaces)
{
	outFaces.clear();
	for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next())
	{
		outFaces.push_back(TopoDS::Face(exp.Current()));
	}
	return !outFaces.empty();
}

bool outerWireOfFace(const TopoDS_Face& face, TopoDS_Wire& outWire)
{
	outWire = BRepTools::OuterWire(face);
	return !outWire.IsNull();
}

std::size_t assignPointStepForSelectedFaces(const std::size_t pointCount, const std::size_t selectedFaceCount,
											const std::size_t maxPointsPerFace)
{
	const std::size_t budget = std::max<std::size_t>(1U, selectedFaceCount) * maxPointsPerFace;
	return std::max<std::size_t>(1U, pointCount / budget);
}

std::size_t effectiveAssignPointsPerFace(const std::size_t faceCount, const std::size_t maxPointsPerFace,
										 const bool hasSelection)
{
	if (hasSelection)
	{
		return maxPointsPerFace;
	}
	// 全工件时按面数摊薄预算，避免 faceCount*maxPerFace 采样爆炸
	const std::size_t autoCap = std::max<std::size_t>(30U, 8000U / std::max<std::size_t>(1U, faceCount));
	return std::min(maxPointsPerFace, autoCap);
}

int parallelWorkerCount()
{
#ifdef _OPENMP
	return std::max(1, omp_get_max_threads());
#else
	return 1;
#endif
}

struct FaceUpdateWorkItem
{
	FaceUpdateReport report;
	TopoDS_Face newFace;
	bool replaceFace = false;
};

double bboxDiagonalMm(const TopoDS_Shape& shape)
{
	Bnd_Box box;
	BRepBndLib::Add(shape, box);
	if (box.IsVoid())
	{
		return 0.0;
	}
	Standard_Real x0 = 0.0;
	Standard_Real y0 = 0.0;
	Standard_Real z0 = 0.0;
	Standard_Real x1 = 0.0;
	Standard_Real y1 = 0.0;
	Standard_Real z1 = 0.0;
	box.Get(x0, y0, z0, x1, y1, z1);
	const double dx = x1 - x0;
	const double dy = y1 - y0;
	const double dz = z1 - z0;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool faceReplaceBboxSane(const TopoDS_Face& originalFace, const TopoDS_Face& newFace)
{
	constexpr double kMaxFaceDiagScale = 3.0;
	constexpr double kMaxFaceDiagExtraMm = 500.0;
	const double oldDiag = bboxDiagonalMm(originalFace);
	const double newDiag = bboxDiagonalMm(newFace);
	return newDiag <= oldDiag * kMaxFaceDiagScale + kMaxFaceDiagExtraMm;
}

void expandBndBox(Bnd_Box& box, const double marginMm)
{
	if (box.IsVoid() || marginMm <= 0.0)
	{
		return;
	}
	Standard_Real x0 = 0.0;
	Standard_Real y0 = 0.0;
	Standard_Real z0 = 0.0;
	Standard_Real x1 = 0.0;
	Standard_Real y1 = 0.0;
	Standard_Real z1 = 0.0;
	box.Get(x0, y0, z0, x1, y1, z1);
	box.Update(x0 - marginMm, y0 - marginMm, z0 - marginMm, x1 + marginMm, y1 + marginMm, z1 + marginMm);
}

bool bndBoxContainsPoint(const Bnd_Box& box, const gp_Pnt& pt)
{
	if (box.IsVoid())
	{
		return false;
	}
	Standard_Real x0 = 0.0;
	Standard_Real y0 = 0.0;
	Standard_Real z0 = 0.0;
	Standard_Real x1 = 0.0;
	Standard_Real y1 = 0.0;
	Standard_Real z1 = 0.0;
	box.Get(x0, y0, z0, x1, y1, z1);
	return pt.X() >= x0 && pt.X() <= x1 && pt.Y() >= y0 && pt.Y() <= y1 && pt.Z() >= z0 && pt.Z() <= z1;
}

void smoothPoleDeltaAccum(std::vector<PoleDeltaAccum>& accum, const int nUPoles, const int nVPoles, const int passes)
{
	if (passes <= 0 || nUPoles < 1 || nVPoles < 1)
	{
		return;
	}
	const auto cellIndex = [nVPoles](const int iu, const int iv)
	{ return static_cast<std::size_t>((iu - 1) * nVPoles + (iv - 1)); };
	for (int pass = 0; pass < passes; ++pass)
	{
		std::vector<PoleDeltaAccum> next = accum;
		for (int iu = 1; iu <= nUPoles; ++iu)
		{
			for (int iv = 1; iv <= nVPoles; ++iv)
			{
				const PoleDeltaAccum& center = accum[cellIndex(iu, iv)];
				if (center.weightSum < 1e-12)
				{
					continue;
				}
				Vec3 moveSum = center.delta / center.weightSum;
				int neighborCount = 1;
				for (int di = -1; di <= 1; ++di)
				{
					for (int dj = -1; dj <= 1; ++dj)
					{
						if (di == 0 && dj == 0)
						{
							continue;
						}
						const int iuN = iu + di;
						const int ivN = iv + dj;
						if (iuN < 1 || iuN > nUPoles || ivN < 1 || ivN > nVPoles)
						{
							continue;
						}
						const PoleDeltaAccum& neighbor = accum[cellIndex(iuN, ivN)];
						if (neighbor.weightSum < 1e-12)
						{
							continue;
						}
						moveSum += neighbor.delta / neighbor.weightSum;
						++neighborCount;
					}
				}
				const Vec3 smoothed = moveSum / static_cast<double>(neighborCount);
				PoleDeltaAccum& outCell = next[cellIndex(iu, iv)];
				outCell.delta = smoothed * center.weightSum;
			}
		}
		accum.swap(next);
	}
}

struct AggregatedUvSample
{
	double u = 0.0;
	double v = 0.0;
	Vec3 delta = Vec3::Zero();
};

std::vector<AggregatedUvSample> aggregateOutliersToUvGrid(const std::vector<OutlierSample>& outliers,
														  const Handle(Geom_BSplineSurface) & surf, const int gridU,
														  const int gridV)
{
	const int nu = std::max(4, gridU);
	const int nv = std::max(4, gridV);
	const int uDeg = surf->UDegree();
	const int vDeg = surf->VDegree();
	const Standard_Real uMin = surf->UKnot(uDeg + 1);
	const Standard_Real uMax = surf->UKnot(surf->NbUKnots() - uDeg);
	const Standard_Real vMin = surf->VKnot(vDeg + 1);
	const Standard_Real vMax = surf->VKnot(surf->NbVKnots() - vDeg);
	const double uSpan = std::max(1e-9, static_cast<double>(uMax - uMin));
	const double vSpan = std::max(1e-9, static_cast<double>(vMax - vMin));

	struct CellAccum
	{
		Vec3 deltaSum = Vec3::Zero();
		double uSum = 0.0;
		double vSum = 0.0;
		int count = 0;
	};
	std::vector<CellAccum> cells(static_cast<std::size_t>(nu * nv));

	for (const OutlierSample& sample : outliers)
	{
		gp_Pnt onSurf;
		surf->D0(sample.u, sample.v, onSurf);
		const Vec3 delta = Vec3(sample.localTarget.X(), sample.localTarget.Y(), sample.localTarget.Z()) -
						   Vec3(onSurf.X(), onSurf.Y(), onSurf.Z());

		int cu = static_cast<int>(std::floor((sample.u - static_cast<double>(uMin)) / uSpan * static_cast<double>(nu)));
		int cv = static_cast<int>(std::floor((sample.v - static_cast<double>(vMin)) / vSpan * static_cast<double>(nv)));
		cu = std::max(0, std::min(nu - 1, cu));
		cv = std::max(0, std::min(nv - 1, cv));
		CellAccum& cell = cells[static_cast<std::size_t>(cu * nv + cv)];
		cell.deltaSum += delta;
		cell.uSum += sample.u;
		cell.vSum += sample.v;
		++cell.count;
	}

	std::vector<AggregatedUvSample> aggregated;
	aggregated.reserve(static_cast<std::size_t>(nu * nv));
	for (int cu = 0; cu < nu; ++cu)
	{
		for (int cv = 0; cv < nv; ++cv)
		{
			const CellAccum& cell = cells[static_cast<std::size_t>(cu * nv + cv)];
			if (cell.count <= 0)
			{
				continue;
			}
			const double inv = 1.0 / static_cast<double>(cell.count);
			aggregated.push_back({cell.uSum * inv, cell.vSum * inv, cell.deltaSum * inv});
		}
	}
	return aggregated;
}

double pointToFaceDistanceMm(const gp_Pnt& point, const TopoDS_Face& face, gp_Vec* outFaceNormal = nullptr)
{
	BRepAdaptor_Surface surf(face, true);
	GeomAPI_ProjectPointOnSurf proj(point, BRep_Tool::Surface(face));
	if (proj.NbPoints() <= 0)
	{
		return std::numeric_limits<double>::max();
	}
	const double dist = std::sqrt(proj.LowerDistance());
	if (outFaceNormal != nullptr && proj.NbPoints() > 0)
	{
		Standard_Real u = 0.0;
		Standard_Real v = 0.0;
		proj.LowerDistanceParameters(u, v);
		gp_Pnt p;
		gp_Vec d1u;
		gp_Vec d1v;
		surf.D1(u, v, p, d1u, d1v);
		gp_Vec n = d1u.Crossed(d1v);
		if (face.Orientation() == TopAbs_REVERSED)
		{
			n.Reverse();
		}
		*outFaceNormal = n;
	}
	return dist;
}

struct FaceAssignTarget
{
	int faceIndex = -1;
	Bnd_Box bbox;
};

void assignScanPointsParallel(const std::vector<float>& scanXyz, const std::vector<float>& scanNormalsNxNyNz,
							  const bool hasScanNormals, const std::vector<TopoDS_Face>& faces,
							  const std::vector<FaceAssignTarget>& assignTargets, const std::size_t pointStep,
							  const double faceBandMm, const double normalThresholdDeg,
							  std::vector<std::vector<Vec3>>& facePoints)
{
	const std::size_t n = scanXyz.size() / 3U;
	std::vector<std::size_t> sampleIndices;
	sampleIndices.reserve(n / std::max<std::size_t>(1U, pointStep) + 1U);
	for (std::size_t pi = 0; pi < n; pi += pointStep)
	{
		sampleIndices.push_back(pi);
	}
	const int sampleCount = static_cast<int>(sampleIndices.size());
	const int faceCount = static_cast<int>(faces.size());
	const int workerCount = parallelWorkerCount();
	std::vector<std::vector<std::vector<Vec3>>> shards(static_cast<std::size_t>(workerCount));
	for (auto& shard : shards)
	{
		shard.resize(static_cast<std::size_t>(faceCount));
	}

#ifdef _OPENMP
#pragma omp parallel
#endif
	{
		const int workerId =
#ifdef _OPENMP
			omp_get_thread_num();
#else
			0;
#endif
		std::vector<std::vector<Vec3>>& localPoints = shards[static_cast<std::size_t>(workerId)];

#ifdef _OPENMP
#pragma omp for schedule(dynamic, 128)
#endif
		for (int si = 0; si < sampleCount; ++si)
		{
			const std::size_t pi = sampleIndices[static_cast<std::size_t>(si)];
			const std::size_t b = pi * 3U;
			const gp_Pnt pt(scanXyz[b], scanXyz[b + 1U], scanXyz[b + 2U]);
			Vec3 scanNormal;
			if (hasScanNormals)
			{
				scanNormal = Vec3(scanNormalsNxNyNz[b], scanNormalsNxNyNz[b + 1U], scanNormalsNxNyNz[b + 2U]);
			}

			int bestFace = -1;
			double bestDist = std::numeric_limits<double>::max();
			for (const FaceAssignTarget& target : assignTargets)
			{
				if (!bndBoxContainsPoint(target.bbox, pt))
				{
					continue;
				}
				const double dist = pointToFaceDistanceMm(pt, faces[static_cast<std::size_t>(target.faceIndex)]);
				if (dist > faceBandMm || dist >= bestDist)
				{
					continue;
				}
				bestFace = target.faceIndex;
				bestDist = dist;
			}
			if (bestFace < 0)
			{
				continue;
			}
			const TopoDS_Face& matchedFace = faces[static_cast<std::size_t>(bestFace)];
			if (hasScanNormals && scanNormal.norm() > 1e-9)
			{
				gp_Vec faceNormal;
				(void)pointToFaceDistanceMm(pt, matchedFace, &faceNormal);
				if (faceNormal.Magnitude() > 1e-9)
				{
					const Vec3 fn(faceNormal.X(), faceNormal.Y(), faceNormal.Z());
					if (normalAngleDeg(scanNormal, fn) > normalThresholdDeg)
					{
						continue;
					}
				}
			}
			localPoints[static_cast<std::size_t>(bestFace)].emplace_back(pt.X(), pt.Y(), pt.Z());
		}
	}

	for (const auto& shard : shards)
	{
		for (std::size_t fi = 0; fi < facePoints.size(); ++fi)
		{
			const auto& src = shard[fi];
			if (!src.empty())
			{
				auto& dst = facePoints[fi];
				dst.insert(dst.end(), src.begin(), src.end());
			}
		}
	}
}

void computeFaceDeviations(const std::vector<Vec3>& assignedPts, const TopoDS_Face& face, double& outAvg,
						   double& outMax)
{
	outAvg = 0.0;
	outMax = 0.0;
	if (assignedPts.empty())
	{
		return;
	}
	double sum = 0.0;
	for (const auto& p : assignedPts)
	{
		const double d = pointToFaceDistanceMm(gp_Pnt(p(0), p(1), p(2)), face);
		sum += d;
		outMax = std::max(outMax, d);
	}
	outAvg = sum / static_cast<double>(assignedPts.size());
}

bool fitPlaneFromPoints(const std::vector<Vec3>& pts, gp_Pln& outPln)
{
	if (pts.size() < 3U)
	{
		return false;
	}
	Vec3 centroid = Vec3::Zero();
	for (const auto& p : pts)
	{
		centroid += p;
	}
	centroid /= static_cast<double>(pts.size());

	Eigen::MatrixXd A(static_cast<int>(pts.size()), 3);
	for (std::size_t i = 0; i < pts.size(); ++i)
	{
		A.row(static_cast<int>(i)) = (pts[i] - centroid).transpose();
	}
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV);
	Vec3 normal = svd.matrixV().col(2).normalized();
	if (normal.norm() < 1e-9)
	{
		return false;
	}
	const gp_Pnt origin(centroid(0), centroid(1), centroid(2));
	const gp_Dir dir(normal(0), normal(1), normal(2));
	outPln = gp_Pln(origin, dir);
	return true;
}

bool fitCylinderFromPoints(const std::vector<Vec3>& pts, gp_Ax1& outAxis, double& outRadius)
{
	if (pts.size() < 6U)
	{
		return false;
	}
	Vec3 centroid = Vec3::Zero();
	for (const auto& p : pts)
	{
		centroid += p;
	}
	centroid /= static_cast<double>(pts.size());

	Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
	for (const auto& p : pts)
	{
		const Vec3 q = p - centroid;
		cov += q * q.transpose();
	}
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
	Vec3 axisDir = es.eigenvectors().col(2).normalized();

	double radiusSum = 0.0;
	for (const auto& p : pts)
	{
		const Vec3 rel = p - centroid;
		const Vec3 axial = axisDir * axisDir.dot(rel);
		const Vec3 radial = rel - axial;
		radiusSum += radial.norm();
	}
	outRadius = radiusSum / static_cast<double>(pts.size());
	if (outRadius < 1e-6)
	{
		return false;
	}
	outAxis = gp_Ax1(gp_Pnt(centroid(0), centroid(1), centroid(2)), gp_Dir(axisDir(0), axisDir(1), axisDir(2)));
	return true;
}

bool makeFaceWithWire(const Handle(Geom_Surface) & surface, const TopoDS_Wire& wire, TopoDS_Face& outFace)
{
	BRepBuilderAPI_MakeFace maker(surface, wire, true);
	if (!maker.IsDone())
	{
		return false;
	}
	outFace = maker.Face();
	return !outFace.IsNull();
}

bool makePlanarFaceWithWire(const gp_Pln& pln, const TopoDS_Wire& wire, TopoDS_Face& outFace)
{
	BRepBuilderAPI_MakeFace maker(pln, wire, true);
	if (!maker.IsDone())
	{
		return false;
	}
	outFace = maker.Face();
	return !outFace.IsNull();
}

bool makeFreeformFaceFromPoints(const std::vector<Vec3>& pts, const TopoDS_Wire& wire, TopoDS_Face& outFace)
{
	if (pts.size() < 12U)
	{
		return false;
	}

	Vec3 centroid = Vec3::Zero();
	for (const auto& p : pts)
	{
		centroid += p;
	}
	centroid /= static_cast<double>(pts.size());

	Eigen::MatrixXd A(3, static_cast<int>(pts.size()));
	for (std::size_t i = 0; i < pts.size(); ++i)
	{
		A.col(static_cast<int>(i)) = pts[i] - centroid;
	}
	Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU);
	const Vec3 uDir = svd.matrixU().col(0).normalized();
	const Vec3 vDir = svd.matrixU().col(1).normalized();

	double uMin = std::numeric_limits<double>::max();
	double uMax = std::numeric_limits<double>::lowest();
	double vMin = uMin;
	double vMax = uMax;
	for (const auto& p : pts)
	{
		const Vec3 rel = p - centroid;
		const double u = rel.dot(uDir);
		const double v = rel.dot(vDir);
		uMin = std::min(uMin, u);
		uMax = std::max(uMax, u);
		vMin = std::min(vMin, v);
		vMax = std::max(vMax, v);
	}

	const int gridU = 10;
	const int gridV = 10;
	const double du = (uMax - uMin) / static_cast<double>(gridU - 1);
	const double dv = (vMax - vMin) / static_cast<double>(gridV - 1);
	if (du < 1e-9 || dv < 1e-9)
	{
		return false;
	}

	TColgp_Array2OfPnt grid(1, gridU, 1, gridV);
	std::vector<int> counts(static_cast<std::size_t>(gridU * gridV), 0);
	for (const auto& p : pts)
	{
		const Vec3 rel = p - centroid;
		const double u = rel.dot(uDir);
		const double v = rel.dot(vDir);
		int iu = static_cast<int>(std::floor((u - uMin) / du));
		int iv = static_cast<int>(std::floor((v - vMin) / dv));
		iu = std::max(1, std::min(gridU, iu + 1));
		iv = std::max(1, std::min(gridV, iv + 1));
		const int flat = (iu - 1) * gridV + (iv - 1);
		if (counts[static_cast<std::size_t>(flat)] == 0)
		{
			grid.SetValue(iu, iv, gp_Pnt(p(0), p(1), p(2)));
		}
		else
		{
			const gp_Pnt prev = grid.Value(iu, iv);
			grid.SetValue(iu, iv, gp_Pnt((prev.X() + p(0)) * 0.5, (prev.Y() + p(1)) * 0.5, (prev.Z() + p(2)) * 0.5));
		}
		++counts[static_cast<std::size_t>(flat)];
	}

	for (int iu = 1; iu <= gridU; ++iu)
	{
		for (int iv = 1; iv <= gridV; ++iv)
		{
			const int flat = (iu - 1) * gridV + (iv - 1);
			if (counts[static_cast<std::size_t>(flat)] == 0)
			{
				const double u = uMin + static_cast<double>(iu - 1) * du;
				const double v = vMin + static_cast<double>(iv - 1) * dv;
				const Vec3 p = centroid + u * uDir + v * vDir;
				grid.SetValue(iu, iv, gp_Pnt(p(0), p(1), p(2)));
			}
		}
	}

	try
	{
		GeomAPI_PointsToBSplineSurface approx(grid, Approx_ChordLength, 3, 8, GeomAbs_C2, 1.0);
		if (approx.IsDone())
		{
			Handle(Geom_BSplineSurface) surf = approx.Surface();
			if (!surf.IsNull() && makeFaceWithWire(surf, wire, outFace))
			{
				return true;
			}
		}
	}
	catch (...)
	{
	}

	gp_Pln fallback;
	if (!fitPlaneFromPoints(pts, fallback))
	{
		return false;
	}
	return makePlanarFaceWithWire(fallback, wire, outFace);
}

bool refitFaceFromPoints(const TopoDS_Face& oldFace, const std::vector<Vec3>& assignedPts, TopoDS_Face& outFace,
						 FaceUpdateAction& outAction)
{
	outAction = FaceUpdateAction::Unchanged;
	if (assignedPts.size() < 3U)
	{
		outAction = FaceUpdateAction::SkippedNoPoints;
		return false;
	}

	TopoDS_Wire wire;
	if (!outerWireOfFace(oldFace, wire))
	{
		return false;
	}

	BRepAdaptor_Surface adapt(oldFace, true);
	const GeomAbs_SurfaceType surfType = adapt.GetType();

	if (surfType == GeomAbs_Plane)
	{
		gp_Pln pln;
		if (!fitPlaneFromPoints(assignedPts, pln))
		{
			return false;
		}
		if (!makePlanarFaceWithWire(pln, wire, outFace))
		{
			return false;
		}
		outAction = FaceUpdateAction::PlaneRefit;
		return true;
	}

	if (surfType == GeomAbs_Cylinder)
	{
		gp_Ax1 axis;
		double radius = 0.0;
		if (!fitCylinderFromPoints(assignedPts, axis, radius))
		{
			return false;
		}
		const Standard_Real u1 = adapt.FirstUParameter();
		const Standard_Real u2 = adapt.LastUParameter();
		const Standard_Real v1 = adapt.FirstVParameter();
		const Standard_Real v2 = adapt.LastVParameter();
		const gp_Ax3 cylAx3(axis.Location(), gp_Dir(axis.Direction()));
		const gp_Cylinder cyl(cylAx3, radius);
		Handle(Geom_CylindricalSurface) surf = new Geom_CylindricalSurface(cyl);
		BRepBuilderAPI_MakeFace maker(surf, u1, u2, v1, v2, 1e-6);
		if (!maker.IsDone())
		{
			return false;
		}
		outFace = maker.Face();
		if (outFace.IsNull())
		{
			return false;
		}
		outAction = FaceUpdateAction::CylinderRefit;
		return true;
	}

	if (!makeFreeformFaceFromPoints(assignedPts, wire, outFace))
	{
		return false;
	}
	outAction = FaceUpdateAction::FreeformRefit;
	return true;
}

double pointToShapeMaxDistance(const std::vector<float>& xyz, const TopoDS_Shape& shape, double& outAvgDist)
{
	const std::size_t n = xyz.size() / 3U;
	if (n == 0U || shape.IsNull())
	{
		outAvgDist = 0.0;
		return 0.0;
	}

	double maxDist = 0.0;
	double sumDist = 0.0;
	const std::size_t maxSamples = 512U;
	const std::size_t step = std::max<std::size_t>(1U, n / maxSamples);
	std::size_t sampleCount = 0U;

	for (std::size_t i = 0; i < n; i += step)
	{
		const std::size_t b = i * 3U;
		const gp_Pnt pt(xyz[b], xyz[b + 1U], xyz[b + 2U]);
		BRepBuilderAPI_MakeVertex vertexMaker(pt);
		if (!vertexMaker.IsDone())
		{
			continue;
		}
		BRepExtrema_DistShapeShape dist(vertexMaker.Vertex(), shape);
		if (!dist.IsDone() || dist.NbSolution() < 1)
		{
			continue;
		}
		const double d = dist.Value();
		maxDist = std::max(maxDist, d);
		sumDist += d;
		++sampleCount;
	}

	outAvgDist = (sampleCount > 0U) ? (sumDist / static_cast<double>(sampleCount)) : 0.0;
	return maxDist;
}

// 面类型调整函数：用扫描点拟合新几何，创建面时保留原始wire

bool fitConeFromPoints(const std::vector<Vec3>& pts, gp_Ax1& outAxis, double& outHalfAngle, double& outRadius)
{
	if (pts.size() < 6U)
	{
		return false;
	}
	// 用PCA估算轴线方向
	Vec3 centroid = Vec3::Zero();
	for (const auto& p : pts)
	{
		centroid += p;
	}
	centroid /= static_cast<double>(pts.size());

	Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
	for (const auto& p : pts)
	{
		const Vec3 q = p - centroid;
		cov += q * q.transpose();
	}
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
	Vec3 axisDir = es.eigenvectors().col(2).normalized();

	// 计算每个点到轴线的径向距离和轴向位置
	double radiusSum = 0.0;
	double minAxial = std::numeric_limits<double>::max();
	double maxAxial = std::numeric_limits<double>::lowest();
	for (const auto& p : pts)
	{
		const Vec3 rel = p - centroid;
		const double axial = axisDir.dot(rel);
		const Vec3 radial = rel - axisDir * axial;
		radiusSum += radial.norm();
		minAxial = std::min(minAxial, axial);
		maxAxial = std::max(maxAxial, axial);
	}
	outRadius = radiusSum / static_cast<double>(pts.size());
	if (outRadius < 1e-6)
	{
		return false;
	}
	// 半角近似：用径向距离变化估算
	outHalfAngle = std::atan2(outRadius, std::max(1.0, maxAxial - minAxial));
	outAxis = gp_Ax1(gp_Pnt(centroid(0), centroid(1), centroid(2)), gp_Dir(axisDir(0), axisDir(1), axisDir(2)));
	return true;
}

bool fitSphereFromPoints(const std::vector<Vec3>& pts, gp_Pnt& outCenter, double& outRadius)
{
	if (pts.size() < 4U)
	{
		return false;
	}
	Vec3 centroid = Vec3::Zero();
	for (const auto& p : pts)
	{
		centroid += p;
	}
	centroid /= static_cast<double>(pts.size());

	double radiusSum = 0.0;
	for (const auto& p : pts)
	{
		radiusSum += (p - centroid).norm();
	}
	outRadius = radiusSum / static_cast<double>(pts.size());
	if (outRadius < 1e-6)
	{
		return false;
	}
	outCenter = gp_Pnt(centroid(0), centroid(1), centroid(2));
	return true;
}

bool fitToroidFromPoints(const std::vector<Vec3>& pts, gp_Ax1& outAxis, double& outMajorRadius, double& outMinorRadius)
{
	if (pts.size() < 8U)
	{
		return false;
	}
	Vec3 centroid = Vec3::Zero();
	for (const auto& p : pts)
	{
		centroid += p;
	}
	centroid /= static_cast<double>(pts.size());

	Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
	for (const auto& p : pts)
	{
		const Vec3 q = p - centroid;
		cov += q * q.transpose();
	}
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);
	Vec3 axisDir = es.eigenvectors().col(2).normalized();

	// 计算每个点到轴线的距离
	std::vector<double> ringDistances;
	for (const auto& p : pts)
	{
		const Vec3 rel = p - centroid;
		const double axial = axisDir.dot(rel);
		const Vec3 radial = rel - axisDir * axial;
		ringDistances.push_back(radial.norm());
	}
	std::sort(ringDistances.begin(), ringDistances.end());
	const double medianRadius = ringDistances[ringDistances.size() / 2];

	// 主半径为中位数径向距离，次半径为点到环面中心线的平均偏差
	double minorSum = 0.0;
	for (const auto& p : pts)
	{
		const Vec3 rel = p - centroid;
		const double axial = axisDir.dot(rel);
		const Vec3 radial = rel - axisDir * axial;
		const double ringDist = radial.norm();
		minorSum += std::abs(ringDist - medianRadius);
	}
	outMajorRadius = medianRadius;
	outMinorRadius = minorSum / static_cast<double>(pts.size());
	if (outMajorRadius < 1e-6 || outMinorRadius < 1e-6)
	{
		return false;
	}
	outAxis = gp_Ax1(gp_Pnt(centroid(0), centroid(1), centroid(2)), gp_Dir(axisDir(0), axisDir(1), axisDir(2)));
	return true;
}

bool adjustPlaneFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out, FaceUpdateAction& action)
{
	gp_Pln fittedPln;
	if (!fitPlaneFromPoints(pts, fittedPln))
	{
		return false;
	}
	TopoDS_Wire wire;
	if (!outerWireOfFace(face, wire))
	{
		return false;
	}
	if (!makePlanarFaceWithWire(fittedPln, wire, out))
	{
		return false;
	}
	action = FaceUpdateAction::PlaneAdjusted;
	return true;
}

bool adjustCylinderFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out,
						FaceUpdateAction& action)
{
	gp_Ax1 axis;
	double radius = 0.0;
	if (!fitCylinderFromPoints(pts, axis, radius))
	{
		return false;
	}
	BRepAdaptor_Surface adapt(face, true);
	const Standard_Real u1 = adapt.FirstUParameter();
	const Standard_Real u2 = adapt.LastUParameter();
	const Standard_Real v1 = adapt.FirstVParameter();
	const Standard_Real v2 = adapt.LastVParameter();
	const gp_Ax3 cylAx3(axis.Location(), gp_Dir(axis.Direction()));
	const gp_Cylinder cyl(cylAx3, radius);
	Handle(Geom_CylindricalSurface) surf = new Geom_CylindricalSurface(cyl);
	BRepBuilderAPI_MakeFace maker(surf, u1, u2, v1, v2, 1e-6);
	if (!maker.IsDone())
	{
		return false;
	}
	out = maker.Face();
	if (out.IsNull())
	{
		return false;
	}
	action = FaceUpdateAction::CylinderAdjusted;
	return true;
}

bool adjustConeFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out, FaceUpdateAction& action)
{
	gp_Ax1 axis;
	double halfAngle = 0.0;
	double radius = 0.0;
	if (!fitConeFromPoints(pts, axis, halfAngle, radius))
	{
		return false;
	}
	BRepAdaptor_Surface adapt(face, true);
	const Standard_Real u1 = adapt.FirstUParameter();
	const Standard_Real u2 = adapt.LastUParameter();
	const Standard_Real v1 = adapt.FirstVParameter();
	const Standard_Real v2 = adapt.LastVParameter();
	const gp_Ax3 coneAx3(axis.Location(), gp_Dir(axis.Direction()));
	const gp_Cone cone(coneAx3, halfAngle, radius);
	Handle(Geom_ConicalSurface) surf = new Geom_ConicalSurface(cone);
	BRepBuilderAPI_MakeFace maker(surf, u1, u2, v1, v2, 1e-6);
	if (!maker.IsDone())
	{
		return false;
	}
	out = maker.Face();
	if (out.IsNull())
	{
		return false;
	}
	action = FaceUpdateAction::ConeAdjusted;
	return true;
}

bool adjustSphereFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out, FaceUpdateAction& action)
{
	gp_Pnt center;
	double radius = 0.0;
	if (!fitSphereFromPoints(pts, center, radius))
	{
		return false;
	}
	BRepAdaptor_Surface adapt(face, true);
	const Standard_Real u1 = adapt.FirstUParameter();
	const Standard_Real u2 = adapt.LastUParameter();
	const Standard_Real v1 = adapt.FirstVParameter();
	const Standard_Real v2 = adapt.LastVParameter();
	const gp_Sphere sphere(gp_Ax3(center, gp_Dir(0, 0, 1)), radius);
	const Handle(Geom_SphericalSurface) surf = new Geom_SphericalSurface(sphere);
	BRepBuilderAPI_MakeFace maker(surf, u1, u2, v1, v2, 1e-6);
	if (!maker.IsDone())
	{
		return false;
	}
	out = maker.Face();
	if (out.IsNull())
	{
		return false;
	}
	action = FaceUpdateAction::SphereAdjusted;
	return true;
}

bool adjustToroidFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, TopoDS_Face& out, FaceUpdateAction& action)
{
	gp_Ax1 axis;
	double majorRadius = 0.0;
	double minorRadius = 0.0;
	if (!fitToroidFromPoints(pts, axis, majorRadius, minorRadius))
	{
		return false;
	}
	BRepAdaptor_Surface adapt(face, true);
	const Standard_Real u1 = adapt.FirstUParameter();
	const Standard_Real u2 = adapt.LastUParameter();
	const Standard_Real v1 = adapt.FirstVParameter();
	const Standard_Real v2 = adapt.LastVParameter();
	const gp_Torus torus(gp_Ax3(axis.Location(), gp_Dir(axis.Direction())), majorRadius, minorRadius);
	Handle(Geom_ToroidalSurface) surf = new Geom_ToroidalSurface(torus);
	BRepBuilderAPI_MakeFace maker(surf, u1, u2, v1, v2, 1e-6);
	if (!maker.IsDone())
	{
		return false;
	}
	out = maker.Face();
	if (out.IsNull())
	{
		return false;
	}
	action = FaceUpdateAction::ToroidAdjusted;
	return true;
}

Handle(Geom_BSplineSurface) extractBsplineSurface(const Handle(Geom_Surface) & geomSurf)
{
	Handle(Geom_BSplineSurface) direct = Handle(Geom_BSplineSurface)::DownCast(geomSurf);
	if (!direct.IsNull())
	{
		return direct;
	}
	const Handle(Geom_RectangularTrimmedSurface) trimmed = Handle(Geom_RectangularTrimmedSurface)::DownCast(geomSurf);
	if (!trimmed.IsNull())
	{
		return Handle(Geom_BSplineSurface)::DownCast(trimmed->BasisSurface());
	}
	return Handle(Geom_BSplineSurface)();
}

gp_Pnt pointInSurfaceLocalFrame(const gp_Pnt& worldPoint, const TopLoc_Location& faceLocation)
{
	if (faceLocation.IsIdentity())
	{
		return worldPoint;
	}
	return worldPoint.Transformed(faceLocation.Transformation().Inverted());
}

double bsplineAdjustThresholdMm(const TemplateBrepUpdateParams& params)
{
	if (params.bsplineAdjustThresholdMm > 0.0)
	{
		return params.bsplineAdjustThresholdMm;
	}
	// 与质量门控 maxAllowedDeviationMm 分离：门控放宽不应抬高 BSpline 调整触发阈值
	return 0.5;
}

double bsplineMaxPoleMoveMm(const TemplateBrepUpdateParams& params, const double thresholdMm)
{
	if (params.bsplineMaxPoleMoveMm > 0.0)
	{
		return params.bsplineMaxPoleMoveMm;
	}
	return std::max(3.0 * thresholdMm, 1.0);
}

bool computeBsplineBasis(const TColStd_Array1OfReal& knotSeq, const int nPoles, const int degree, const double t,
						 int& outSpanIndex, std::vector<double>& outBasis)
{
	const int order = degree + 1;
	const int nKnots = knotSeq.Length();
	const int lo = knotSeq.Lower();
	const int hi = knotSeq.Upper();

	std::vector<double> knots(static_cast<std::size_t>(nKnots));
	for (int i = 0; i < nKnots; ++i)
	{
		knots[static_cast<std::size_t>(i)] = knotSeq.Value(lo + i);
	}

	const double kFirst = knots[0];
	const double kLast = knots[static_cast<std::size_t>(nKnots - 1)];
	const double eps = 1e-10;
	double tt = t;
	if (tt < kFirst)
	{
		tt = kFirst + eps;
	}
	if (tt > kLast)
	{
		tt = kLast - eps;
	}

	int span = -1;
	for (int i = degree; i < nKnots - 1 - degree; ++i)
	{
		if (tt >= knots[static_cast<std::size_t>(i)] && tt < knots[static_cast<std::size_t>(i + 1)])
		{
			span = i;
			break;
		}
	}
	if (span < 0)
	{
		if (std::abs(tt - kLast) < eps)
		{
			for (int i = nKnots - 2; i >= degree; --i)
			{
				if (knots[static_cast<std::size_t>(i)] < knots[static_cast<std::size_t>(i + 1)] - eps)
				{
					span = i;
					break;
				}
			}
		}
		if (span < 0)
		{
			return false;
		}
	}

	outSpanIndex = span;
	outBasis.resize(static_cast<std::size_t>(order), 0.0);
	outBasis[0] = 1.0;

	std::vector<double> left(static_cast<std::size_t>(order), 0.0);
	std::vector<double> right(static_cast<std::size_t>(order), 0.0);

	for (int j = 1; j < order; ++j)
	{
		left[static_cast<std::size_t>(j)] = tt - knots[static_cast<std::size_t>(span + 1 - j)];
		right[static_cast<std::size_t>(j)] = knots[static_cast<std::size_t>(span + j)] - tt;

		double saved = 0.0;
		for (int r = 0; r < j; ++r)
		{
			const double temp = outBasis[static_cast<std::size_t>(r)] /
								(right[static_cast<std::size_t>(r + 1)] + left[static_cast<std::size_t>(j - r)]);
			outBasis[static_cast<std::size_t>(r)] = saved + right[static_cast<std::size_t>(r + 1)] * temp;
			saved = left[static_cast<std::size_t>(j - r)] * temp;
		}
		outBasis[static_cast<std::size_t>(j)] = saved;
	}
	return true;
}

bool distributeOutlierDeltaToPoles(const Handle(Geom_BSplineSurface) & surf, const double u, const double v,
								   const Vec3& delta, std::vector<PoleDeltaAccum>& accum, bool* outEvalFailed = nullptr)
{
	if (outEvalFailed)
	{
		*outEvalFailed = false;
	}
	const int nUPoles = surf->NbUPoles();
	const int nVPoles = surf->NbVPoles();
	const int uDeg = surf->UDegree();
	const int vDeg = surf->VDegree();
	const TColStd_Array1OfReal& uKnots = surf->UKnotSequence();
	const TColStd_Array1OfReal& vKnots = surf->VKnotSequence();

	int uSpan = -1;
	std::vector<double> uBasis;
	if (!computeBsplineBasis(uKnots, nUPoles, uDeg, u, uSpan, uBasis))
	{
		if (outEvalFailed)
		{
			*outEvalFailed = true;
		}
		return false;
	}
	int vSpan = -1;
	std::vector<double> vBasis;
	if (!computeBsplineBasis(vKnots, nVPoles, vDeg, v, vSpan, vBasis))
	{
		if (outEvalFailed)
		{
			*outEvalFailed = true;
		}
		return false;
	}

	const int uOrder = uDeg + 1;
	const int vOrder = vDeg + 1;
	const int uFirst = uSpan - uDeg;
	const int vFirst = vSpan - vDeg;

	bool any = false;
	for (int i = 0; i < uOrder; ++i)
	{
		const int iu = uFirst + i + 1;
		if (iu < 1 || iu > nUPoles)
		{
			continue;
		}
		const double wu = uBasis[static_cast<std::size_t>(i)];
		if (wu < 1e-15)
		{
			continue;
		}
		for (int j = 0; j < vOrder; ++j)
		{
			const int iv = vFirst + j + 1;
			if (iv < 1 || iv > nVPoles)
			{
				continue;
			}
			const double wv = vBasis[static_cast<std::size_t>(j)];
			const double w = wu * wv;
			if (w < 1e-15)
			{
				continue;
			}
			PoleDeltaAccum& cell = accum[static_cast<std::size_t>((iu - 1) * nVPoles + (iv - 1))];
			cell.delta += w * delta;
			cell.weightSum += w;
			any = true;
		}
	}
	return any;
}

bool applyPoleDeltaAccum(const Handle(Geom_BSplineSurface) & surf, const std::vector<PoleDeltaAccum>& accum,
						 const double maxMoveMm, int& adjustedPoleCount)
{
	adjustedPoleCount = 0;
	const int nUPoles = surf->NbUPoles();
	const int nVPoles = surf->NbVPoles();
	for (int iu = 1; iu <= nUPoles; ++iu)
	{
		for (int iv = 1; iv <= nVPoles; ++iv)
		{
			const PoleDeltaAccum& cell = accum[static_cast<std::size_t>((iu - 1) * nVPoles + (iv - 1))];
			if (cell.weightSum < 1e-12)
			{
				continue;
			}
			Vec3 move = cell.delta / cell.weightSum;
			const double moveLen = move.norm();
			if (moveLen < 1e-9)
			{
				continue;
			}
			if (moveLen > maxMoveMm)
			{
				move *= maxMoveMm / moveLen;
			}
			gp_Pnt pole = surf->Pole(iu, iv);
			pole.SetX(pole.X() + move(0));
			pole.SetY(pole.Y() + move(1));
			pole.SetZ(pole.Z() + move(2));
			surf->SetPole(iu, iv, pole);
			++adjustedPoleCount;
		}
	}
	return adjustedPoleCount > 0;
}

bool adjustBSplineFace(const TopoDS_Face& face, const std::vector<Vec3>& pts, const TemplateBrepUpdateParams& params,
					   TopoDS_Face& out, FaceUpdateAction& action)
{
	action = FaceUpdateAction::Unchanged;
	TopLoc_Location loc;
	const Handle(Geom_Surface) geomSurf = BRep_Tool::Surface(face, loc);
	Handle(Geom_BSplineSurface) geomBSpline = extractBsplineSurface(geomSurf);
	if (geomBSpline.IsNull())
	{
		return false;
	}
	Handle(Geom_BSplineSurface) surf = Handle(Geom_BSplineSurface)::DownCast(geomBSpline->Copy());
	if (surf.IsNull())
	{
		return false;
	}

	const double threshold = bsplineAdjustThresholdMm(params);
	const double maxMove = bsplineMaxPoleMoveMm(params, threshold);

	std::vector<OutlierSample> outliers;
	outliers.reserve(pts.size());
	for (const Vec3& p : pts)
	{
		const gp_Pnt worldPt(p(0), p(1), p(2));
		const gp_Pnt localPt = pointInSurfaceLocalFrame(worldPt, loc);
		GeomAPI_ProjectPointOnSurf proj(localPt, surf);
		if (proj.NbPoints() <= 0)
		{
			continue;
		}
		const double dist = std::sqrt(proj.LowerDistance());
		if (dist <= threshold)
		{
			continue;
		}
		Standard_Real u = 0.0;
		Standard_Real v = 0.0;
		proj.LowerDistanceParameters(u, v);
		outliers.push_back({u, v, localPt});
	}

	if (outliers.empty())
	{
		out = face;
		return true;
	}

	const int nUPoles = surf->NbUPoles();
	const int nVPoles = surf->NbVPoles();
	std::vector<PoleDeltaAccum> accum(static_cast<std::size_t>(nUPoles * nVPoles));

	const int gridU = std::max(4, params.bsplineUvGridCellsU);
	const int gridV = std::max(4, params.bsplineUvGridCellsV);
	const std::vector<AggregatedUvSample> aggregated = aggregateOutliersToUvGrid(outliers, surf, gridU, gridV);

	for (const AggregatedUvSample& sample : aggregated)
	{
		distributeOutlierDeltaToPoles(surf, sample.u, sample.v, sample.delta, accum);
	}

	smoothPoleDeltaAccum(accum, nUPoles, nVPoles, params.bsplinePoleSmoothPasses);

	int adjustedPoles = 0;
	if (!applyPoleDeltaAccum(surf, accum, maxMove, adjustedPoles))
	{
		out = face;
		return true;
	}

	TopoDS_Wire wire;
	if (!outerWireOfFace(face, wire))
	{
		return false;
	}
	Handle(Geom_Surface) faceSurf = surf;
	if (!loc.IsIdentity())
	{
		faceSurf = Handle(Geom_Surface)::DownCast(surf->Transformed(loc.Transformation()));
		if (faceSurf.IsNull())
		{
			return false;
		}
	}
	if (!makeFaceWithWire(faceSurf, wire, out))
	{
		BRepAdaptor_Surface adapt(face, true);
		BRepBuilderAPI_MakeFace maker(faceSurf, adapt.FirstUParameter(), adapt.LastUParameter(),
									  adapt.FirstVParameter(), adapt.LastVParameter(), 1e-6);
		if (!maker.IsDone())
		{
			return false;
		}
		out = maker.Face();
		if (out.IsNull())
		{
			return false;
		}
	}
	action = FaceUpdateAction::BSplineAdjusted;
	return true;
}

bool adjustFaceGeometryDispatch(const TopoDS_Face& originalFace, const std::vector<Vec3>& assignedPoints,
								const TemplateBrepUpdateParams& params, TopoDS_Face& adjustedFace,
								FaceUpdateAction& action, std::string& outSurfaceTypeName)
{
	action = FaceUpdateAction::Unchanged;
	if (assignedPoints.size() < 3U)
	{
		action = FaceUpdateAction::SkippedNoPoints;
		outSurfaceTypeName = "None";
		return false;
	}

	BRepAdaptor_Surface adapt(originalFace, true);
	const GeomAbs_SurfaceType surfType = adapt.GetType();

	switch (surfType)
	{
	case GeomAbs_Plane:
		outSurfaceTypeName = "Plane";
		return adjustPlaneFace(originalFace, assignedPoints, adjustedFace, action);
	case GeomAbs_Cylinder:
		outSurfaceTypeName = "Cylinder";
		return adjustCylinderFace(originalFace, assignedPoints, adjustedFace, action);
	case GeomAbs_Cone:
		outSurfaceTypeName = "Cone";
		return adjustConeFace(originalFace, assignedPoints, adjustedFace, action);
	case GeomAbs_Sphere:
		outSurfaceTypeName = "Sphere";
		return adjustSphereFace(originalFace, assignedPoints, adjustedFace, action);
	case GeomAbs_Torus:
		outSurfaceTypeName = "Torus";
		return adjustToroidFace(originalFace, assignedPoints, adjustedFace, action);
	case GeomAbs_BSplineSurface:
		outSurfaceTypeName = "BSplineSurface";
		return adjustBSplineFace(originalFace, assignedPoints, params, adjustedFace, action);
	case GeomAbs_SurfaceOfRevolution:
		outSurfaceTypeName = "SurfaceOfRevolution";
		return refitFaceFromPoints(originalFace, assignedPoints, adjustedFace, action);
	case GeomAbs_SurfaceOfExtrusion:
		outSurfaceTypeName = "SurfaceOfExtrusion";
		return refitFaceFromPoints(originalFace, assignedPoints, adjustedFace, action);
	case GeomAbs_OffsetSurface:
		outSurfaceTypeName = "OffsetSurface";
		return refitFaceFromPoints(originalFace, assignedPoints, adjustedFace, action);
	case GeomAbs_OtherSurface:
		outSurfaceTypeName = "OtherSurface";
		return refitFaceFromPoints(originalFace, assignedPoints, adjustedFace, action);
	default:
		outSurfaceTypeName = "Unknown";
		return refitFaceFromPoints(originalFace, assignedPoints, adjustedFace, action);
	}
}

} // namespace

/// 获取面的 OCCT 曲面类型名称（用于调试输出）
static const char* surfaceTypeNameOf(const TopoDS_Face& face)
{
	BRepAdaptor_Surface adapt(face, true);
	switch (adapt.GetType())
	{
	case GeomAbs_Plane:
		return "Plane";
	case GeomAbs_Cylinder:
		return "Cylinder";
	case GeomAbs_Cone:
		return "Cone";
	case GeomAbs_Sphere:
		return "Sphere";
	case GeomAbs_Torus:
		return "Torus";
	case GeomAbs_BSplineSurface:
		return "BSplineSurface";
	case GeomAbs_SurfaceOfRevolution:
		return "SurfaceOfRevolution";
	case GeomAbs_SurfaceOfExtrusion:
		return "SurfaceOfExtrusion";
	case GeomAbs_OffsetSurface:
		return "OffsetSurface";
	case GeomAbs_OtherSurface:
		return "OtherSurface";
	default:
		return "Unknown";
	}
}

bool sampleShapeSurfacePoints(const ShapeHandle& templateShape, const double spacingMm, std::vector<float>& outXyz,
							  std::string* errMsg)
{
	outXyz.clear();
	TopoDS_Shape shape;
	if (!ShapeHandleAccess::nativeShape(templateShape, &shape))
	{
		if (errMsg)
		{
			*errMsg = "template shape access failed";
		}
		return false;
	}

	const double spacing = std::max(0.5, spacingMm);
	std::vector<TopoDS_Face> faces;
	if (!collectFaces(shape, faces))
	{
		if (errMsg)
		{
			*errMsg = "template has no faces";
		}
		return false;
	}

	for (const auto& face : faces)
	{
		BRepAdaptor_Surface surf(face, true);
		const Standard_Real u1 = surf.FirstUParameter();
		const Standard_Real u2 = surf.LastUParameter();
		const Standard_Real v1 = surf.FirstVParameter();
		const Standard_Real v2 = surf.LastVParameter();

		Bnd_Box bb;
		BRepBndLib::Add(face, bb);
		Standard_Real x0 = 0.0;
		Standard_Real y0 = 0.0;
		Standard_Real z0 = 0.0;
		Standard_Real x1 = 0.0;
		Standard_Real y1 = 0.0;
		Standard_Real z1 = 0.0;
		bb.Get(x0, y0, z0, x1, y1, z1);
		const double dx = static_cast<double>(x1 - x0);
		const double dy = static_cast<double>(y1 - y0);
		const double dz = static_cast<double>(z1 - z0);
		const double faceDiagMm = std::sqrt(dx * dx + dy * dy + dz * dz);
		const int grid = std::max(2, std::min(48, static_cast<int>(std::ceil(faceDiagMm / spacing))));
		const int nu = grid;
		const int nv = grid;

		const double uSpan = std::max(1e-6, static_cast<double>(u2 - u1));
		const double vSpan = std::max(1e-6, static_cast<double>(v2 - v1));

		for (int iu = 0; iu <= nu; ++iu)
		{
			for (int iv = 0; iv <= nv; ++iv)
			{
				const double u = u1 + uSpan * static_cast<double>(iu) / static_cast<double>(nu);
				const double v = v1 + vSpan * static_cast<double>(iv) / static_cast<double>(nv);
				gp_Pnt p;
				surf.D0(u, v, p);
				outXyz.push_back(static_cast<float>(p.X()));
				outXyz.push_back(static_cast<float>(p.Y()));
				outXyz.push_back(static_cast<float>(p.Z()));
			}
		}
	}

	if (outXyz.size() < 9U)
	{
		if (errMsg)
		{
			*errMsg = "too few template sample points";
		}
		return false;
	}
	return true;
}

bool updateShapeFromPointCloud(const ShapeHandle& templateShape, const std::vector<float>& scanXyz,
							   const std::vector<float>& scanNormalsNxNyNz, const TemplateBrepUpdateParams& params,
							   TemplateBrepUpdateResult& out, std::string* errMsg)
{
	const Eigen::Isometry3d savedTemplateToScan = out.templateToScan;
	const double savedRmse = out.icpRmseMm;
	out.updatedShape = ShapeHandle{};
	out.perFace.clear();
	out.globalMaxDeviationMm = 0.0;
	out.globalAvgDeviationMm = 0.0;
	out.updatedFaceCount = 0U;
	out.qualityPassed = false;
	out.templateToScan = savedTemplateToScan;
	out.icpRmseMm = savedRmse;

	TopoDS_Shape templateNative;
	if (!ShapeHandleAccess::nativeShape(templateShape, &templateNative))
	{
		if (errMsg)
		{
			*errMsg = "template shape access failed";
		}
		return false;
	}

	const std::size_t n = scanXyz.size() / 3U;
	if (n < 3U)
	{
		if (errMsg)
		{
			*errMsg = "too few scan points";
		}
		return false;
	}

	const bool hasScanNormals = (scanNormalsNxNyNz.size() == scanXyz.size());

	std::vector<TopoDS_Face> faces;
	if (!collectFaces(templateNative, faces))
	{
		if (errMsg)
		{
			*errMsg = "template has no faces";
		}
		return false;
	}

	std::vector<std::vector<Vec3>> facePoints(faces.size());

	std::unordered_set<int> selectedSet;
	for (int idx : params.selectedFaceIndices)
	{
		selectedSet.insert(idx);
	}
	const bool hasSelection = !selectedSet.empty();

	std::vector<FaceAssignTarget> assignTargets;
	assignTargets.reserve(hasSelection ? selectedSet.size() : faces.size());
	if (hasSelection)
	{
		for (int fi : selectedSet)
		{
			if (fi < 0 || static_cast<std::size_t>(fi) >= faces.size())
			{
				continue;
			}
			Bnd_Box bbox;
			BRepBndLib::Add(faces[static_cast<std::size_t>(fi)], bbox);
			expandBndBox(bbox, params.faceBandMm);
			assignTargets.push_back({fi, bbox});
		}
	}
	else
	{
		for (std::size_t fi = 0; fi < faces.size(); ++fi)
		{
			Bnd_Box bbox;
			BRepBndLib::Add(faces[fi], bbox);
			expandBndBox(bbox, params.faceBandMm);
			assignTargets.push_back({static_cast<int>(fi), bbox});
		}
	}

	const std::size_t maxPerFace = std::max<std::size_t>(30U, params.maxAssignPointsPerFace);
	const std::size_t perFaceBudget = effectiveAssignPointsPerFace(faces.size(), maxPerFace, hasSelection);
	const std::size_t budgetFaces = hasSelection ? assignTargets.size() : faces.size();
	const std::size_t pointStep = assignPointStepForSelectedFaces(n, budgetFaces, perFaceBudget);
	const int threadCount = parallelWorkerCount();

	const auto assignStart = std::chrono::steady_clock::now();
	assignScanPointsParallel(scanXyz, scanNormalsNxNyNz, hasScanNormals, faces, assignTargets, pointStep,
							 params.faceBandMm, params.normalThresholdDeg, facePoints);
	const auto assignMs =
		std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - assignStart).count();
	RunLogger::info("[TemplateBrepUpdate] assign scan points done, step=" + std::to_string(pointStep) +
					" perFaceBudget=" + std::to_string(perFaceBudget) + " selective=" + (hasSelection ? "yes" : "no") +
					" threads=" + std::to_string(threadCount) + " ms=" + std::to_string(assignMs));

	if (hasSelection)
	{
		std::size_t loggedFaces = 0U;
		for (int fi : selectedSet)
		{
			if (fi < 0 || static_cast<std::size_t>(fi) >= facePoints.size())
			{
				continue;
			}
			RunLogger::info("[TemplateBrepUpdate] faceAssign fi=" + std::to_string(fi) +
							" pts=" + std::to_string(facePoints[static_cast<std::size_t>(fi)].size()) +
							" need>=" + std::to_string(params.minPointsPerFace));
			if (++loggedFaces >= 16U)
			{
				break;
			}
		}
	}

	std::size_t updatedCount = 0U;
	out.perFace.resize(faces.size());
	std::vector<FaceUpdateWorkItem> workItems(faces.size());

	const auto faceUpdateStart = std::chrono::steady_clock::now();
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
	for (int fi = 0; fi < static_cast<int>(faces.size()); ++fi)
	{
		FaceUpdateWorkItem& item = workItems[static_cast<std::size_t>(fi)];
		FaceUpdateReport& report = item.report;
		report.faceIndex = fi;
		report.assignedPoints = facePoints[static_cast<std::size_t>(fi)].size();
		report.surfaceTypeName = surfaceTypeNameOf(faces[static_cast<std::size_t>(fi)]);

		if (hasSelection && selectedSet.find(fi) == selectedSet.end())
		{
			report.action = FaceUpdateAction::Unchanged;
			continue;
		}

		if (facePoints[static_cast<std::size_t>(fi)].size() < params.minPointsPerFace)
		{
			report.action = FaceUpdateAction::SkippedNoPoints;
			continue;
		}

		TopoDS_Face newFace;
		FaceUpdateAction action = FaceUpdateAction::Unchanged;
		std::string surfTypeName;
		const bool dispatchOk =
			adjustFaceGeometryDispatch(faces[static_cast<std::size_t>(fi)], facePoints[static_cast<std::size_t>(fi)],
									   params, newFace, action, surfTypeName);
		report.surfaceTypeName = surfTypeName;
		if (!dispatchOk)
		{
			report.action = FaceUpdateAction::Unchanged;
			computeFaceDeviations(facePoints[static_cast<std::size_t>(fi)], faces[static_cast<std::size_t>(fi)],
								  report.avgDeviationMm, report.maxDeviationMm);
			continue;
		}

		report.action = action;
		if (action != FaceUpdateAction::Unchanged && action != FaceUpdateAction::SkippedNoPoints)
		{
			item.newFace = newFace;
			item.replaceFace = true;
			computeFaceDeviations(facePoints[static_cast<std::size_t>(fi)], newFace, report.avgDeviationMm,
								  report.maxDeviationMm);
		}
		else
		{
			computeFaceDeviations(facePoints[static_cast<std::size_t>(fi)], faces[static_cast<std::size_t>(fi)],
								  report.avgDeviationMm, report.maxDeviationMm);
		}
	}
	const auto faceUpdateMs =
		std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - faceUpdateStart)
			.count();

	std::size_t skippedNoPoints = 0U;
	std::size_t skippedBadBbox = 0U;
	updatedCount = 0U;
	constexpr double kMaxGlobalDiagScale = 1.5;
	constexpr double kMaxGlobalDiagExtraMm = 500.0;
	const double templateDiag = bboxDiagonalMm(templateNative);
	const double globalBboxLimit = templateDiag * kMaxGlobalDiagScale + kMaxGlobalDiagExtraMm;
	TopoDS_Shape workingShape = templateNative;
	std::vector<TopoDS_Face> workingFaces;
	collectFaces(workingShape, workingFaces);
	for (std::size_t fi = 0; fi < faces.size(); ++fi)
	{
		const FaceUpdateWorkItem& item = workItems[fi];
		out.perFace[fi] = item.report;
		if (item.report.action == FaceUpdateAction::SkippedNoPoints)
		{
			++skippedNoPoints;
		}
		if (!item.replaceFace)
		{
			continue;
		}
		if (!faceReplaceBboxSane(faces[fi], item.newFace))
		{
			++skippedBadBbox;
			out.perFace[fi].action = FaceUpdateAction::Unchanged;
			continue;
		}
		if (fi >= workingFaces.size())
		{
			continue;
		}
		BRepTools_ReShape trialReshaper;
		trialReshaper.Replace(workingFaces[fi], item.newFace);
		const TopoDS_Shape candidate = trialReshaper.Apply(workingShape, TopAbs_SHAPE);
		if (candidate.IsNull())
		{
			++skippedBadBbox;
			out.perFace[fi].action = FaceUpdateAction::Unchanged;
			continue;
		}
		const double candDiag = bboxDiagonalMm(candidate);
		if (templateDiag > 1e-3 && candDiag > globalBboxLimit)
		{
			++skippedBadBbox;
			out.perFace[fi].action = FaceUpdateAction::Unchanged;
			continue;
		}
		workingShape = candidate;
		// 增量更新：直接替换对应面，避免重复遍历整个 shape
		if (fi < workingFaces.size())
		{
			workingFaces[fi] = item.newFace;
		}
		++updatedCount;
	}

	RunLogger::info("[TemplateBrepUpdate] face geometry update done, updated=" + std::to_string(updatedCount) +
					" skippedBbox=" + std::to_string(skippedBadBbox) +
					" skippedNoPts=" + std::to_string(skippedNoPoints) + " threads=" + std::to_string(threadCount) +
					" ms=" + std::to_string(faceUpdateMs));

	TopoDS_Shape reshaped = workingShape;
	TopoDS_Shape fixed = reshaped;
	if (updatedCount > 0U)
	{
		const double diagBeforeFix = bboxDiagonalMm(reshaped);
		Handle(ShapeFix_Shape) fixer = new ShapeFix_Shape(reshaped);
		fixer->Perform();
		const TopoDS_Shape fixerOut = fixer->Shape();
		const double diagAfterFix = bboxDiagonalMm(fixerOut);
		const bool usedShapeFix = !fixerOut.IsNull() && diagAfterFix <= diagBeforeFix * 1.25 + 1.0;
		// ShapeFix 偶发拉远包围盒，导致去心后网格浮点精度丢失而不可见
		if (usedShapeFix)
		{
			fixed = fixerOut;
		}

		constexpr double kMaxGlobalDiagScale = 1.5;
		constexpr double kMaxGlobalDiagExtraMm = 500.0;
		const double outputDiag = bboxDiagonalMm(fixed);
		if (templateDiag > 1e-3 && outputDiag > templateDiag * kMaxGlobalDiagScale + kMaxGlobalDiagExtraMm)
		{
			fixed = templateNative;
			updatedCount = 0U;
			RunLogger::warn("[TemplateBrepUpdate] output bbox exploded (diag=" + std::to_string(outputDiag) +
							"mm vs template=" + std::to_string(templateDiag) + "mm), reverted to template");
		}
	}

	out.updatedShape = ShapeHandleAccess::fromNativeShape(&fixed);
	if (out.updatedShape.isNull())
	{
		if (errMsg)
		{
			*errMsg = "updated shape handle failed";
		}
		return false;
	}

	out.updatedFaceCount = updatedCount;
	out.skippedBadBboxFaceCount = skippedBadBbox;

	if (params.maxAllowedDeviationMm > 0.0)
	{
		const bool selectiveQualityGate = hasSelection || (updatedCount < faces.size());
		if (selectiveQualityGate)
		{
			double sumWeighted = 0.0;
			std::size_t pointCount = 0U;
			out.globalMaxDeviationMm = 0.0;
			for (std::size_t fi = 0; fi < out.perFace.size(); ++fi)
			{
				if (hasSelection && selectedSet.find(static_cast<int>(fi)) == selectedSet.end())
				{
					continue;
				}
				const FaceUpdateReport& report = out.perFace[fi];
				if (updatedCount > 0U && (report.action == FaceUpdateAction::Unchanged ||
										  report.action == FaceUpdateAction::SkippedNoPoints))
				{
					continue;
				}
				if (report.assignedPoints == 0U)
				{
					continue;
				}
				sumWeighted += report.avgDeviationMm * static_cast<double>(report.assignedPoints);
				pointCount += report.assignedPoints;
				out.globalMaxDeviationMm = std::max(out.globalMaxDeviationMm, report.maxDeviationMm);
			}
			out.globalAvgDeviationMm = pointCount > 0U ? sumWeighted / static_cast<double>(pointCount) : 0.0;
		}
		else
		{
			out.globalMaxDeviationMm = pointToShapeMaxDistance(scanXyz, fixed, out.globalAvgDeviationMm);
		}
		out.qualityPassed = (out.globalMaxDeviationMm <= params.maxAllowedDeviationMm);
	}
	else
	{
		out.qualityPassed = true;
	}

	return true;
}

} // namespace geoalgo
