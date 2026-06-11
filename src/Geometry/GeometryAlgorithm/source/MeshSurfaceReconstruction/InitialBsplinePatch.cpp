#include "MeshSurfaceReconstructionInternal.h"

#include "detail/OccIncludes.h"

#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakePolygon.hxx>
#include <Geom_Plane.hxx>
#include <BRepBndLib.hxx>
#include <Bnd_Box.hxx>

#include <algorithm>
#include <cmath>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

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

double polesBBoxDiagonal(const Handle(Geom_BSplineSurface)& surface)
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

enum class BsplineFitReject
{
	None,
	ApproxFailed,
	PoleExploded
};

TColgp_Array2OfPnt downsampleGrid(const TColgp_Array2OfPnt& grid, const int fitN)
{
	const int srcN = grid.UpperRow() - grid.LowerRow();
	TColgp_Array2OfPnt fitGrid(1, fitN + 1, 1, fitN + 1);
	for (int iu = 0; iu <= fitN; ++iu)
	{
		const int srcIu = grid.LowerRow() + (iu * srcN) / fitN;
		for (int iv = 0; iv <= fitN; ++iv)
		{
			const int srcIv = grid.LowerCol() + (iv * srcN) / fitN;
			fitGrid.SetValue(iu + 1, iv + 1, grid.Value(srcIu, srcIv));
		}
	}
	return fitGrid;
}

bool surfaceFitsGrid(
	const Handle(Geom_Surface)& surface,
	const TColgp_Array2OfPnt& grid,
	const double maxErrMm)
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

BsplineFitReject tryFitBsplineSurface(
	const TColgp_Array2OfPnt& grid,
	const int n,
	const double sampleDiag,
	Handle(Geom_BSplineSurface)& outSurface)
{
	const int fitN = std::min(n, 9);
	const TColgp_Array2OfPnt fitGrid = downsampleGrid(grid, fitN);
	const TColgp_Array2OfPnt& useGrid = fitGrid;
	const double gridFitErrMm = std::max(5.0, sampleDiag * 0.06);

	const double tolerances[] = {1.0, std::max(1.0, sampleDiag * 0.015)};
	const Approx_ParametrizationType paramTypes[] = {Approx_IsoParametric, Approx_ChordLength};
	double maxPoleDiag = 0.0;

	for (const double tol : tolerances)
	{
		for (const Approx_ParametrizationType paramType : paramTypes)
		{
			GeomAPI_PointsToBSplineSurface approx(useGrid, paramType, 3, 6, GeomAbs_C1, tol);
			if (!approx.IsDone() || approx.Surface().IsNull())
			{
				continue;
			}
			const double poleDiag = polesBBoxDiagonal(approx.Surface());
			maxPoleDiag = std::max(maxPoleDiag, poleDiag);
			if (poleDiag > sampleDiag * 1.8)
			{
				continue;
			}
			if (!surfaceFitsGrid(approx.Surface(), useGrid, gridFitErrMm))
			{
				continue;
			}
			outSurface = approx.Surface();
			return BsplineFitReject::None;
		}
	}
	outSurface.Nullify();
	if (maxPoleDiag > sampleDiag * 1.8)
	{
		return BsplineFitReject::PoleExploded;
	}
	return BsplineFitReject::ApproxFailed;
}

bool tryMakeFaceFromSurface(
	const Handle(Geom_Surface)& surface,
	const double sampleDiag,
	TopoDS_Face& outFace)
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
	BRepBuilderAPI_MakeFace mkFace(surface, u1, u2, v1, v2, 1e-3);
	if (!mkFace.IsDone())
	{
		return false;
	}
	outFace = mkFace.Face();
	return faceBBoxDiagonal(outFace) <= sampleDiag * 2.0;
}

bool tryPlaneFallbackFace(const TColgp_Array2OfPnt& grid, TopoDS_Face& outFace)
{
	const int uMax = grid.UpperRow();
	const int vMax = grid.UpperCol();
	if (uMax < 2 || vMax < 2)
	{
		return false;
	}
	const gp_Pnt p00 = grid.Value(1, 1);
	const gp_Pnt p10 = grid.Value(uMax, 1);
	const gp_Pnt p11 = grid.Value(uMax, vMax);
	const gp_Pnt p01 = grid.Value(1, vMax);
	const gp_Vec v1(p00, p10);
	const gp_Vec v2(p00, p01);
	gp_Vec normal = v1.Crossed(v2);
	if (normal.Magnitude() < 1e-9)
	{
		normal = gp_Vec(p10, p11).Crossed(gp_Vec(p10, p01));
	}
	if (normal.Magnitude() < 1e-9)
	{
		return false;
	}
	const gp_Pln pln(p00, gp_Dir(normal));

	BRepBuilderAPI_MakePolygon poly;
	poly.Add(p00);
	poly.Add(p10);
	poly.Add(p11);
	poly.Add(p01);
	poly.Close();
	if (!poly.IsDone())
	{
		return false;
	}
	const Handle(Geom_Plane) geomPln = new Geom_Plane(pln);
	BRepBuilderAPI_MakeFace mk(geomPln, poly.Wire(), Standard_True);
	if (!mk.IsDone())
	{
		return false;
	}
	outFace = mk.Face();
	return !outFace.IsNull();
}

} // namespace

bool buildInitialBsplinePatches(
	std::vector<QuadPatch>& patches,
	const MeshSurfaceReconstructParams& params,
	std::string* errMsg)
{
	const int defaultN = std::max(4, std::min(32, params.samplesPerPatchEdge));
	int builtCount = 0;

	for (QuadPatch& patch : patches)
	{
		patch.surface.Nullify();
		patch.face.Nullify();

		if (patch.faceIndices.empty())
		{
			continue;
		}

		const int n = patch.gridN > 0 ? patch.gridN : defaultN;
		const std::size_t expectedSamples = static_cast<std::size_t>((n + 1) * (n + 1) * 3U);
		if (patch.sampleXyz.size() != expectedSamples)
		{
			if (errMsg)
			{
				*errMsg = "patch sample grid size mismatch";
			}
			return false;
		}

		TColgp_Array2OfPnt grid(1, n + 1, 1, n + 1);
		std::size_t idx = 0U;
		for (int iu = 0; iu <= n; ++iu)
		{
			for (int iv = 0; iv <= n; ++iv)
			{
				grid.SetValue(
					iu + 1,
					iv + 1,
					gp_Pnt(
						static_cast<double>(patch.sampleXyz[idx]),
						static_cast<double>(patch.sampleXyz[idx + 1U]),
						static_cast<double>(patch.sampleXyz[idx + 2U])));
				idx += 3U;
			}
		}

		if (!gridHasSpread(grid, 1e-4))
		{
			continue;
		}

		const double sampleDiag = gridBBoxDiagonal(grid);
		bool faceOk = false;

		try
		{
			if (tryFitBsplineSurface(grid, n, sampleDiag, patch.surface) == BsplineFitReject::None)
			{
				faceOk = tryMakeFaceFromSurface(patch.surface, sampleDiag, patch.face);
			}
		}
		catch (...)
		{
			patch.surface.Nullify();
			patch.face.Nullify();
		}

		if (!faceOk)
		{
			patch.surface.Nullify();
			patch.face.Nullify();
			faceOk = tryPlaneFallbackFace(grid, patch.face);
		}

		if (!faceOk)
		{
			continue;
		}

		++builtCount;
	}

	if (builtCount < 1)
	{
		if (errMsg)
		{
			*errMsg = "no valid B-spline patches built";
		}
		return false;
	}
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
