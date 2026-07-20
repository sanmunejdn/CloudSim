/// @file InitialBsplinePatch.cpp
/// @brief InitialBsplinePatch 实现

#include "MeshSurfaceReconstructionInternal.h"
#include "NurbsSurfaceFitting.h"
#include "RunLogger.h"
#include "detail/OccIncludes.h"

#include <algorithm>
#include <cmath>

#include <BRepAdaptor_Surface.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <Bnd_Box.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <TopAbs.hxx>
#include <TopoDS.hxx>

namespace geoalgo
{
namespace meshrecon
{
namespace
{
constexpr double kMaxPoleToSampleRatio = 2.5;
constexpr double kMaxFaceToSampleRatio = 2.5;

double bboxDiagonal(const Bnd_Box& box)
{
	if (box.IsVoid())
	{
		return 0.0;
	}
	double xmin = 0.0;
	double ymin = 0.0;
	double zmin = 0.0;
	double xmax = 0.0;
	double ymax = 0.0;
	double zmax = 0.0;
	box.Get(xmin, ymin, zmin, xmax, ymax, zmax);
	const double dx = xmax - xmin;
	const double dy = ymax - ymin;
	const double dz = zmax - zmin;
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double gridBBoxDiagonal(const TColgp_Array2OfPnt& grid)
{
	Bnd_Box box;
	for (int iu = grid.LowerRow(); iu <= grid.UpperRow(); ++iu)
	{
		for (int iv = grid.LowerCol(); iv <= grid.UpperCol(); ++iv)
		{
			box.Add(grid.Value(iu, iv));
		}
	}
	return bboxDiagonal(box);
}

double polesBBoxDiagonal(const Handle(Geom_BSplineSurface) & surface)
{
	if (surface.IsNull())
	{
		return 0.0;
	}
	Bnd_Box box;
	const TColgp_Array2OfPnt poles = surface->Poles();
	for (int iu = poles.LowerRow(); iu <= poles.UpperRow(); ++iu)
	{
		for (int iv = poles.LowerCol(); iv <= poles.UpperCol(); ++iv)
		{
			box.Add(poles.Value(iu, iv));
		}
	}
	return bboxDiagonal(box);
}

double faceBBoxDiagonal(const TopoDS_Face& face)
{
	if (face.IsNull())
	{
		return 0.0;
	}
	Bnd_Box box;
	BRepBndLib::Add(face, box);
	return bboxDiagonal(box);
}

bool gridHasSpread(const TColgp_Array2OfPnt& grid, const double minSpanMm)
{
	const double diag = gridBBoxDiagonal(grid);
	return diag >= minSpanMm;
}

const char* patchFitRejectReasonText(const PatchFitRejectReason reason)
{
	switch (reason)
	{
	case PatchFitRejectReason::Approx:
		return "approxFail";
	case PatchFitRejectReason::Pole:
		return "pole";
	case PatchFitRejectReason::FitGrid:
		return "fitGrid";
	case PatchFitRejectReason::FullGrid:
		return "fullGrid";
	case PatchFitRejectReason::MakeFace:
		return "makeFace";
	case PatchFitRejectReason::Skipped:
		return "skipped";
	default:
		return "none";
	}
}

TColgp_Array2OfPnt downsampleGridRect(const TColgp_Array2OfPnt& grid, const int fitNu, const int fitNv)
{
	const int srcNu = grid.UpperRow() - grid.LowerRow();
	const int srcNv = grid.UpperCol() - grid.LowerCol();
	TColgp_Array2OfPnt fitGrid(1, fitNu + 1, 1, fitNv + 1);
	for (int iu = 0; iu <= fitNu; ++iu)
	{
		const int srcIu = grid.LowerRow() + (srcNu > 0 ? (iu * srcNu) / fitNu : 0);
		for (int iv = 0; iv <= fitNv; ++iv)
		{
			const int srcIv = grid.LowerCol() + (srcNv > 0 ? (iv * srcNv) / fitNv : 0);
			fitGrid.SetValue(iu + 1, iv + 1, grid.Value(srcIu, srcIv));
		}
	}
	return fitGrid;
}

int resolveFitEdgeCount(const int gridEdgeCount, const double uvSpanMm, const MeshSurfaceReconstructParams& params)
{
	int fitN = gridEdgeCount;
	if (params.fitUvSpacingMm > 1e-6 && uvSpanMm > 1e-6)
	{
		fitN = static_cast<int>(std::ceil(uvSpanMm / params.fitUvSpacingMm));
		fitN = std::max(4, std::min(gridEdgeCount, fitN));
	}
	if (params.maxFitGridPerEdge > 0)
	{
		fitN = std::min(fitN, params.maxFitGridPerEdge);
	}
	return std::max(1, fitN);
}

bool surfaceFitsGrid(const Handle(Geom_Surface) & surface, const TColgp_Array2OfPnt& grid, const double maxErrMm)
{
	if (surface.IsNull())
	{
		return false;
	}
	double u1 = 0.0;
	double u2 = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;
	surface->Bounds(u1, u2, v1, v2);
	const int nu = grid.UpperRow() - grid.LowerRow();
	const int nv = grid.UpperCol() - grid.LowerCol();
	for (int iu = 0; iu <= nu; ++iu)
	{
		const double u = u1 + (nu > 0 ? static_cast<double>(iu) / static_cast<double>(nu) : 0.0) * (u2 - u1);
		for (int iv = 0; iv <= nv; ++iv)
		{
			const double v = v1 + (nv > 0 ? static_cast<double>(iv) / static_cast<double>(nv) : 0.0) * (v2 - v1);
			const gp_Pnt onSurf = surface->Value(u, v);
			const gp_Pnt data = grid.Value(grid.LowerRow() + iu, grid.LowerCol() + iv);
			const double dx = onSurf.X() - data.X();
			const double dy = onSurf.Y() - data.Y();
			const double dz = onSurf.Z() - data.Z();
			if (std::sqrt(dx * dx + dy * dy + dz * dz) > maxErrMm)
			{
				return false;
			}
		}
	}
	return true;
}

void updateDeepestReject(PatchFitRejectReason& deepest, const PatchFitRejectReason stage)
{
	if (static_cast<int>(stage) > static_cast<int>(deepest))
	{
		deepest = stage;
	}
}

NurbsFitMode toNurbsFitMode(const MeshSurfaceNurbsFitMode mode)
{
	switch (mode)
	{
	case MeshSurfaceNurbsFitMode::Interpolate:
		return NurbsFitMode::Interpolate;
	case MeshSurfaceNurbsFitMode::ApproxCentripetal:
		return NurbsFitMode::ApproxCentripetal;
	case MeshSurfaceNurbsFitMode::ApproxCentripetalFixedCtrlpts:
		return NurbsFitMode::ApproxCentripetalFixedCtrlpts;
	case MeshSurfaceNurbsFitMode::ApproxFixedCtrlpts:
	default:
		return NurbsFitMode::ApproxFixedCtrlpts;
	}
}

int resolveControlPointCount(const int computedCount, const int fitEdgeCount, const int degree,
							 const MeshSurfaceReconstructParams& params)
{
	const int maxFromFit = std::max(degree + 1, static_cast<int>(0.75 * fitEdgeCount));
	int ctrl = computedCount;
	if (ctrl <= degree + 1)
	{
		ctrl = std::max(params.minControlPointsPerDirection, degree + 1);
	}
	ctrl = std::min(ctrl, maxFromFit);
	const int densityCtrl =
		static_cast<int>(std::lround(params.controlPointDensityFactor * static_cast<double>(fitEdgeCount + 1) * 2.0));
	ctrl = std::min(ctrl, std::max(degree + 1, densityCtrl));
	return std::max(degree + 1, ctrl);
}

bool surfaceFitsGridSubsampled(const Handle(Geom_Surface) & surface, const TColgp_Array2OfPnt& grid, const int targetNu,
							   const int targetNv, const double maxErrMm)
{
	if (targetNu < grid.UpperRow() - grid.LowerRow() || targetNv < grid.UpperCol() - grid.LowerCol())
	{
		const TColgp_Array2OfPnt sub = downsampleGridRect(grid, targetNu, targetNv);
		return surfaceFitsGrid(surface, sub, maxErrMm);
	}
	return surfaceFitsGrid(surface, grid, maxErrMm);
}

PatchFitRejectReason tryFitNurbsSurface(const TColgp_Array2OfPnt& fullGrid, const int gridNu, const int gridNv,
										const double uvUSpanMm, const double uvVSpanMm, const double sampleDiag,
										const int computedCtrlPtsU, const int computedCtrlPtsV,
										const MeshSurfaceReconstructParams& params,
										Handle(Geom_BSplineSurface) & outSurface)
{
	const int fitNu = resolveFitEdgeCount(gridNu, uvUSpanMm, params);
	const int fitNv = resolveFitEdgeCount(gridNv, uvVSpanMm, params);
	const TColgp_Array2OfPnt fitGrid = downsampleGridRect(fullGrid, fitNu, fitNv);

	const int ctrlU = resolveControlPointCount(computedCtrlPtsU, fitNu, params.nurbsDegreeU, params);
	const int ctrlV = resolveControlPointCount(computedCtrlPtsV, fitNv, params.nurbsDegreeV, params);
	const NurbsFitMode fitMode = toNurbsFitMode(params.fitMode);

	PatchFitRejectReason deepest = PatchFitRejectReason::None;

	const auto validateSurface = [&](const Handle(Geom_BSplineSurface) & surface, const double fitTolFrac,
									 const double fullTolFrac, const double maxPoleRatio) -> bool
	{
		if (surface.IsNull())
		{
			updateDeepestReject(deepest, PatchFitRejectReason::Approx);
			return false;
		}
		const double poleDiag = polesBBoxDiagonal(surface);
		if (poleDiag > sampleDiag * maxPoleRatio)
		{
			updateDeepestReject(deepest, PatchFitRejectReason::Pole);
			return false;
		}
		const double fitErrMm = std::max(2.0, sampleDiag * fitTolFrac);
		if (!surfaceFitsGrid(surface, fitGrid, fitErrMm))
		{
			updateDeepestReject(deepest, PatchFitRejectReason::FitGrid);
			return false;
		}
		const double validateErrMm = std::max(2.0, sampleDiag * fullTolFrac);
		if (fitNu < gridNu || fitNv < gridNv)
		{
			if (!surfaceFitsGridSubsampled(surface, fullGrid, fitNu, fitNv, validateErrMm))
			{
				updateDeepestReject(deepest, PatchFitRejectReason::FullGrid);
				return false;
			}
		}
		else if (!surfaceFitsGrid(surface, fullGrid, validateErrMm))
		{
			updateDeepestReject(deepest, PatchFitRejectReason::FullGrid);
			return false;
		}
		outSurface = surface;
		return true;
	};

	const NurbsFitMode fitModes[] = {
		fitMode,
		NurbsFitMode::ApproxCentripetal,
		NurbsFitMode::ApproxFixedCtrlpts,
		NurbsFitMode::ApproxCentripetalFixedCtrlpts,
	};
	const double fitTolFracs[] = {0.04, 0.06, 0.08, 0.12};
	const double fullTolFracs[] = {0.04, 0.06, 0.08, 0.12, 0.16};
	for (const NurbsFitMode modeTry : fitModes)
	{
		for (const double fitTolFrac : fitTolFracs)
		{
			for (const double fullTolFrac : fullTolFracs)
			{
				if (fullTolFrac + 1e-9 < fitTolFrac)
				{
					continue;
				}
				Handle(Geom_BSplineSurface) nurbsSurface;
				if (!fitNurbsSurfaceFromGrid(fitGrid, ctrlU, ctrlV, modeTry, params.nurbsDegreeU, params.nurbsDegreeV,
											 nurbsSurface))
				{
					updateDeepestReject(deepest, PatchFitRejectReason::Approx);
					continue;
				}
				if (validateSurface(nurbsSurface, fitTolFrac, fullTolFrac, kMaxPoleToSampleRatio))
				{
					return PatchFitRejectReason::None;
				}
			}
		}
	}

	struct FitCombo
	{
		int degMin;
		int degMax;
		GeomAbs_Shape continuity;
		double tol;
	};
	const FitCombo combos[] = {
		{3, 3, GeomAbs_C0, 1.0}, {3, 3, GeomAbs_C0, std::max(1.0, sampleDiag * 0.015)},
		{3, 3, GeomAbs_C1, 1.0}, {3, 3, GeomAbs_C1, std::max(1.0, sampleDiag * 0.015)},
		{3, 5, GeomAbs_C0, 1.0}, {3, 5, GeomAbs_C1, 1.0},
	};

	const auto tryCombo = [&](const FitCombo& combo, const Approx_ParametrizationType paramType,
							  const double maxPoleRatio, const double fitTolFrac, const double fullTolFrac) -> bool
	{
		GeomAPI_PointsToBSplineSurface approx(fitGrid, paramType, combo.degMin, combo.degMax, combo.continuity,
											  combo.tol);
		if (!approx.IsDone() || approx.Surface().IsNull())
		{
			updateDeepestReject(deepest, PatchFitRejectReason::Approx);
			return false;
		}
		return validateSurface(approx.Surface(), fitTolFrac, fullTolFrac, maxPoleRatio);
	};

	const double fullTolFracsGeom[] = {0.04, 0.06, 0.08, 0.12, 0.16};
	const double chordFitTolFracs[] = {0.04, 0.06, 0.08, 0.12};
	for (const double fullTolFrac : fullTolFracsGeom)
	{
		for (const double fitTolFrac : chordFitTolFracs)
		{
			if (fullTolFrac + 1e-9 < fitTolFrac)
			{
				continue;
			}
			for (const auto& combo : combos)
			{
				if (tryCombo(combo, Approx_Centripetal, kMaxPoleToSampleRatio, fitTolFrac, fullTolFrac))
				{
					return PatchFitRejectReason::None;
				}
			}
		}
	}
	for (const double fullTolFrac : fullTolFracsGeom)
	{
		for (const double fitTolFrac : chordFitTolFracs)
		{
			if (fullTolFrac + 1e-9 < fitTolFrac)
			{
				continue;
			}
			for (const auto& combo : combos)
			{
				if (tryCombo(combo, Approx_ChordLength, kMaxPoleToSampleRatio, fitTolFrac, fullTolFrac))
				{
					return PatchFitRejectReason::None;
				}
			}
		}
	}

	const double isoMaxPoleRatio = 3.0;
	for (const double fullTolFrac : fullTolFracsGeom)
	{
		for (const auto& combo : combos)
		{
			if (tryCombo(combo, Approx_IsoParametric, isoMaxPoleRatio, 0.06, fullTolFrac))
			{
				return PatchFitRejectReason::None;
			}
		}
	}

	outSurface.Nullify();
	return deepest == PatchFitRejectReason::None ? PatchFitRejectReason::Approx : deepest;
}

bool tryMakeFaceFromCornerUvWire(const Handle(Geom_Surface) & surface, const int gridNu, const int gridNv,
								 const double maxFaceDiag, TopoDS_Face& outFace)
{
	if (surface.IsNull() || gridNu < 1 || gridNv < 1)
	{
		return false;
	}
	double u1 = 0.0;
	double u2 = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;
	surface->Bounds(u1, u2, v1, v2);
	const auto cornerOnSurf = [&](const int iu, const int iv) -> gp_Pnt
	{
		const double fu = static_cast<double>(iu) / static_cast<double>(gridNu);
		const double fv = static_cast<double>(iv) / static_cast<double>(gridNv);
		return surface->Value(u1 + fu * (u2 - u1), v1 + fv * (v2 - v1));
	};

	BRepBuilderAPI_MakePolygon poly;
	poly.Add(cornerOnSurf(0, 0));
	poly.Add(cornerOnSurf(gridNu, 0));
	poly.Add(cornerOnSurf(gridNu, gridNv));
	poly.Add(cornerOnSurf(0, gridNv));
	poly.Close();
	if (!poly.IsDone())
	{
		return false;
	}
	BRepBuilderAPI_MakeFace mkFace(surface, poly.Wire(), Standard_True);
	if (!mkFace.IsDone())
	{
		return false;
	}
	outFace = mkFace.Face();
	return faceBBoxDiagonal(outFace) <= maxFaceDiag;
}

bool tryMakeFaceFromSurface(const Handle(Geom_Surface) & surface, const double sampleDiag, const int gridNu,
							const int gridNv, TopoDS_Face& outFace)
{
	if (surface.IsNull())
	{
		return false;
	}
	const double maxFaceDiag = sampleDiag * kMaxFaceToSampleRatio;
	double u1 = 0.0;
	double u2 = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;
	surface->Bounds(u1, u2, v1, v2);
	BRepBuilderAPI_MakeFace mkFace(surface, u1, u2, v1, v2, 1e-3);
	if (mkFace.IsDone())
	{
		outFace = mkFace.Face();
		if (faceBBoxDiagonal(outFace) <= maxFaceDiag)
		{
			return true;
		}
	}
	return tryMakeFaceFromCornerUvWire(surface, gridNu, gridNv, maxFaceDiag, outFace);
}

gp_Pnt meshVertexPnt(const IndexedMeshLite& mesh, const int vi)
{
	const std::size_t b = static_cast<std::size_t>(vi) * 3U;
	return gp_Pnt(static_cast<double>(mesh.vertices[b]), static_cast<double>(mesh.vertices[b + 1U]),
				  static_cast<double>(mesh.vertices[b + 2U]));
}

bool triangleHasArea(const gp_Pnt& a, const gp_Pnt& b, const gp_Pnt& c)
{
	const gp_Vec ab(a, b);
	const gp_Vec ac(a, c);
	return ab.Crossed(ac).SquareMagnitude() > 1e-18;
}

bool buildMeshFallbackFaces(const IndexedMeshLite& mesh, const QuadPatch& patch, std::vector<TopoDS_Face>& outFaces)
{
	outFaces.clear();
	outFaces.reserve(patch.faceIndices.size());
	for (const int fi : patch.faceIndices)
	{
		if (fi < 0)
		{
			continue;
		}
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		if (b + 2U >= mesh.faces.size())
		{
			continue;
		}
		const gp_Pnt p0 = meshVertexPnt(mesh, mesh.faces[b]);
		const gp_Pnt p1 = meshVertexPnt(mesh, mesh.faces[b + 1U]);
		const gp_Pnt p2 = meshVertexPnt(mesh, mesh.faces[b + 2U]);
		if (!triangleHasArea(p0, p1, p2))
		{
			continue;
		}
		BRepBuilderAPI_MakePolygon poly;
		poly.Add(p0);
		poly.Add(p1);
		poly.Add(p2);
		poly.Close();
		if (!poly.IsDone())
		{
			continue;
		}
		BRepBuilderAPI_MakeFace mkFace(poly.Wire(), Standard_False);
		if (!mkFace.IsDone())
		{
			continue;
		}
		outFaces.push_back(mkFace.Face());
	}
	return !outFaces.empty();
}

double sampleDiagFromXyz(const std::vector<float>& xyz)
{
	Bnd_Box box;
	for (std::size_t i = 0U; i + 2U < xyz.size(); i += 3U)
	{
		box.Add(gp_Pnt(xyz[i], xyz[i + 1U], xyz[i + 2U]));
	}
	return bboxDiagonal(box);
}

gp_Vec computePatchMeshNormal(const IndexedMeshLite& mesh, const std::vector<int>& faceIndices)
{
	gp_Vec sum(0.0, 0.0, 0.0);
	for (const int fi : faceIndices)
	{
		if (fi < 0)
		{
			continue;
		}
		const std::size_t b = static_cast<std::size_t>(fi) * 3U;
		if (b + 2U >= mesh.faces.size())
		{
			continue;
		}
		const gp_Pnt p0 = meshVertexPnt(mesh, mesh.faces[b]);
		const gp_Pnt p1 = meshVertexPnt(mesh, mesh.faces[b + 1U]);
		const gp_Pnt p2 = meshVertexPnt(mesh, mesh.faces[b + 2U]);
		if (!triangleHasArea(p0, p1, p2))
		{
			continue;
		}
		const gp_Vec ab(p0, p1);
		const gp_Vec ac(p0, p2);
		sum += ab.Crossed(ac);
	}
	if (sum.SquareMagnitude() < 1e-24)
	{
		return gp_Vec(0.0, 0.0, 1.0);
	}
	sum.Normalize();
	return sum;
}

gp_Vec faceOutwardNormalAtMid(const TopoDS_Face& face)
{
	if (face.IsNull())
	{
		return gp_Vec(0.0, 0.0, 1.0);
	}
	BRepAdaptor_Surface surf(face, Standard_True);
	const double uMid = 0.5 * (surf.FirstUParameter() + surf.LastUParameter());
	const double vMid = 0.5 * (surf.FirstVParameter() + surf.LastVParameter());
	gp_Pnt p;
	gp_Vec du;
	gp_Vec dv;
	surf.D1(uMid, vMid, p, du, dv);
	gp_Vec n = du.Crossed(dv);
	if (face.Orientation() == TopAbs_REVERSED)
	{
		n.Reverse();
	}
	if (n.SquareMagnitude() < 1e-24)
	{
		return gp_Vec(0.0, 0.0, 1.0);
	}
	n.Normalize();
	return n;
}

bool orientPatchFaceToMesh(const IndexedMeshLite& mesh, const QuadPatch& patch, TopoDS_Face& face)
{
	if (face.IsNull() || patch.faceIndices.empty())
	{
		return false;
	}
	const gp_Vec meshNormal = computePatchMeshNormal(mesh, patch.faceIndices);
	const gp_Vec faceNormal = faceOutwardNormalAtMid(face);
	if (meshNormal.Dot(faceNormal) >= 0.0)
	{
		return false;
	}
	face = TopoDS::Face(face.Reversed());
	return true;
}

} // namespace

bool buildInitialBsplinePatches(const IndexedMeshLite& mesh, std::vector<QuadPatch>& patches,
								const MeshSurfaceReconstructParams& params, std::string* errMsg)
{
	const int defaultN = std::max(4, params.samplesPerPatchEdge);
	int builtCount = 0;
	int flippedCount = 0;
	int patchIdx = 0;

	for (QuadPatch& patch : patches)
	{
		patch.surface.Nullify();
		patch.face.Nullify();
		patch.meshFallback = false;
		patch.meshFallbackFaces.clear();
		patch.fitRejectReason = PatchFitRejectReason::None;

		if (patch.faceIndices.empty())
		{
			patch.fitRejectReason = PatchFitRejectReason::Skipped;
			++patchIdx;
			continue;
		}

		const int nu = patch.gridNu > 0 ? patch.gridNu : (patch.gridN > 0 ? patch.gridN : defaultN);
		const int nv = patch.gridNv > 0 ? patch.gridNv : nu;
		const std::size_t expectedSamples = static_cast<std::size_t>((nu + 1) * (nv + 1) * 3U);
		if (patch.sampleXyz.size() != expectedSamples)
		{
			if (errMsg)
			{
				*errMsg = "patch sample grid size mismatch";
			}
			return false;
		}

		TColgp_Array2OfPnt grid(1, nu + 1, 1, nv + 1);
		std::size_t idx = 0U;
		for (int iu = 0; iu <= nu; ++iu)
		{
			for (int iv = 0; iv <= nv; ++iv)
			{
				grid.SetValue(iu + 1, iv + 1,
							  gp_Pnt(static_cast<double>(patch.sampleXyz[idx]),
									 static_cast<double>(patch.sampleXyz[idx + 1U]),
									 static_cast<double>(patch.sampleXyz[idx + 2U])));
				idx += 3U;
			}
		}

		if (!gridHasSpread(grid, 1e-4))
		{
			patch.fitRejectReason = PatchFitRejectReason::Skipped;
			++patchIdx;
			continue;
		}

		const double sampleDiag = gridBBoxDiagonal(grid);
		bool faceOk = false;

		try
		{
			const PatchFitRejectReason fitReason =
				tryFitNurbsSurface(grid, nu, nv, patch.uvUSpanMm, patch.uvVSpanMm, sampleDiag, patch.computedCtrlPtsU,
								   patch.computedCtrlPtsV, params, patch.surface);
			if (fitReason == PatchFitRejectReason::None)
			{
				faceOk = tryMakeFaceFromSurface(patch.surface, sampleDiag, nu, nv, patch.face);
				if (faceOk)
				{
					if (orientPatchFaceToMesh(mesh, patch, patch.face))
					{
						++flippedCount;
					}
				}
				else
				{
					patch.fitRejectReason = PatchFitRejectReason::MakeFace;
				}
			}
			else
			{
				patch.fitRejectReason = fitReason;
			}
		}
		catch (...)
		{
			patch.surface.Nullify();
			patch.face.Nullify();
			patch.fitRejectReason = PatchFitRejectReason::Approx;
		}

		if (!faceOk)
		{
			patch.surface.Nullify();
			patch.face.Nullify();
			patch.meshFallback = buildMeshFallbackFaces(mesh, patch, patch.meshFallbackFaces);
			faceOk = patch.meshFallback;
		}

		if (patch.fitRejectReason != PatchFitRejectReason::None)
		{
			RunLogger::info(std::string("patch ") + std::to_string(patchIdx) +
							" fit reject: " + patchFitRejectReasonText(patch.fitRejectReason) +
							(patch.meshFallback ? " (meshFallback)" : ""));
		}

		if (!faceOk)
		{
			++patchIdx;
			continue;
		}

		++builtCount;
		++patchIdx;
	}

	if (builtCount < 1)
	{
		if (errMsg)
		{
			*errMsg = "no valid patches built";
		}
		return false;
	}
	if (flippedCount > 0)
	{
		RunLogger::info(std::string("NURBS fit: oriented ") + std::to_string(flippedCount) + " / " +
						std::to_string(builtCount) + " patches to mesh normals");
	}
	return true;
}

bool rebuildPatchFace(const IndexedMeshLite& mesh, QuadPatch& patch)
{
	if (patch.meshFallback)
	{
		return !patch.meshFallbackFaces.empty();
	}
	if (patch.surface.IsNull())
	{
		return !patch.face.IsNull();
	}
	if (patch.sampleXyz.size() < 9U)
	{
		return !patch.face.IsNull();
	}
	const double sampleDiag = sampleDiagFromXyz(patch.sampleXyz);
	const int gridNu = patch.gridNu > 0 ? patch.gridNu : (patch.gridN > 0 ? patch.gridN : 1);
	const int gridNv = patch.gridNv > 0 ? patch.gridNv : gridNu;
	TopoDS_Face face;
	if (!tryMakeFaceFromSurface(patch.surface, sampleDiag, gridNu, gridNv, face))
	{
		return !patch.face.IsNull();
	}
	(void)orientPatchFaceToMesh(mesh, patch, face);
	patch.face = face;
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
