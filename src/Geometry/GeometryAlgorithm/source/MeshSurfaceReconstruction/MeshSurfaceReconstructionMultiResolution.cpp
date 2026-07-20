/// @file MeshSurfaceReconstructionMultiResolution.cpp
/// @brief MeshSurfaceReconstructionMultiResolution 实现

#include "MeshSurfaceReconstructionInternal.h"
#include "NurbsSurfaceFitting.h"
#include "RunLogger.h"

#include <algorithm>
#include <cmath>
#include <vector>

#include <Geom_BSplineSurface.hxx>
#include <gp_Pnt.hxx>

namespace geoalgo
{
namespace meshrecon
{
namespace
{
double surfaceSampleDeviation(const Handle(Geom_BSplineSurface) & surface, const std::vector<float>& sampleXyz)
{
	if (surface.IsNull() || sampleXyz.size() < 9U)
	{
		return 1e30;
	}
	double u1 = 0.0;
	double u2 = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;
	surface->Bounds(u1, u2, v1, v2);
	double sum = 0.0;
	int count = 0;
	for (std::size_t i = 0U; i + 2U < sampleXyz.size(); i += 3U)
	{
		const gp_Pnt target(static_cast<double>(sampleXyz[i]), static_cast<double>(sampleXyz[i + 1U]),
							static_cast<double>(sampleXyz[i + 2U]));
		double best = 1e30;
		for (int su = 0; su <= 4; ++su)
		{
			for (int sv = 0; sv <= 4; ++sv)
			{
				const double fu = u1 + (u2 - u1) * static_cast<double>(su) / 4.0;
				const double fv = v1 + (v2 - v1) * static_cast<double>(sv) / 4.0;
				const gp_Pnt p = surface->Value(fu, fv);
				best = std::min(best, p.Distance(target));
			}
		}
		sum += best;
		++count;
	}
	return count > 0 ? sum / static_cast<double>(count) : 1e30;
}

int countSurfaceCtrlPts(const Handle(Geom_BSplineSurface) & surface)
{
	if (surface.IsNull())
	{
		return 0;
	}
	const TColgp_Array2OfPnt poles = surface->Poles();
	return (poles.UpperRow() - poles.LowerRow() + 1) * (poles.UpperCol() - poles.LowerCol() + 1);
}

bool sampleSurfaceToGrid(const Handle(Geom_BSplineSurface) & surface, const int nu, const int nv,
						 TColgp_Array2OfPnt& outGrid)
{
	if (surface.IsNull() || nu < 1 || nv < 1)
	{
		return false;
	}
	double u1 = 0.0;
	double u2 = 0.0;
	double v1 = 0.0;
	double v2 = 0.0;
	surface->Bounds(u1, u2, v1, v2);
	outGrid = TColgp_Array2OfPnt(1, nu + 1, 1, nv + 1);
	for (int iu = 0; iu <= nu; ++iu)
	{
		for (int iv = 0; iv <= nv; ++iv)
		{
			const double fu = u1 + (u2 - u1) * static_cast<double>(iu) / static_cast<double>(nu);
			const double fv = v1 + (v2 - v1) * static_cast<double>(iv) / static_cast<double>(nv);
			outGrid.SetValue(iu + 1, iv + 1, surface->Value(fu, fv));
		}
	}
	return true;
}

} // namespace

bool applyMultiResolutionFit(const IndexedMeshLite& mesh, std::vector<QuadPatch>& patches,
							 const MeshSurfaceReconstructParams& params, MeshSurfaceReconstructReport* report,
							 std::string* errMsg)
{
	if (!params.enableMultiResolutionFit || params.multiResolutionLayers < 1)
	{
		return true;
	}

	int reducedCount = 0;
	double ctrlSum = 0.0;
	int ctrlPatchCount = 0;

	for (QuadPatch& patch : patches)
	{
		if (patch.surface.IsNull() || patch.meshFallback || patch.sampleXyz.empty())
		{
			continue;
		}

		const int nu = patch.gridNu > 0 ? patch.gridNu : patch.gridN;
		const int nv = patch.gridNv > 0 ? patch.gridNv : nu;
		if (nu < 1 || nv < 1)
		{
			continue;
		}

		const double baseDev = surfaceSampleDeviation(patch.surface, patch.sampleXyz);
		const int baseCtrl = countSurfaceCtrlPts(patch.surface);
		Handle(Geom_BSplineSurface) bestSurface = patch.surface;
		int bestCtrl = baseCtrl;

		for (int layer = 0; layer < params.multiResolutionLayers; ++layer)
		{
			TColgp_Array2OfPnt layerGrid;
			if (!sampleSurfaceToGrid(bestSurface, nu, nv, layerGrid))
			{
				break;
			}

			const double densityScale =
				std::pow(std::max(0.15, params.multiResolutionDensityScale), static_cast<double>(layer + 1));
			MeshSurfaceReconstructParams layerParams = params;
			layerParams.controlPointDensityFactor = std::max(0.12, params.controlPointDensityFactor * densityScale);

			const int ctrlU =
				resolveControlPointCountFromFitGrid(nu + 1, params.nurbsDegreeU, layerParams.controlPointDensityFactor,
													params.minControlPointsPerDirection);
			const int ctrlV =
				resolveControlPointCountFromFitGrid(nv + 1, params.nurbsDegreeV, layerParams.controlPointDensityFactor,
													params.minControlPointsPerDirection);

			Handle(Geom_BSplineSurface) layerSurface;
			if (!fitNurbsSurfaceFromGrid(layerGrid, ctrlU, ctrlV, nurbsFitModeFromMeshSurface(params.fitMode),
										 params.nurbsDegreeU, params.nurbsDegreeV, layerSurface) ||
				layerSurface.IsNull())
			{
				break;
			}

			const double layerDev = surfaceSampleDeviation(layerSurface, patch.sampleXyz);
			const int layerCtrl = countSurfaceCtrlPts(layerSurface);
			const double devTol = std::max(0.05, baseDev * 1.15);
			if (layerDev <= devTol && layerCtrl < bestCtrl)
			{
				bestSurface = layerSurface;
				bestCtrl = layerCtrl;
			}
		}

		if (bestCtrl < baseCtrl)
		{
			patch.surface = bestSurface;
			(void)rebuildPatchFace(mesh, patch);
			++reducedCount;
		}

		ctrlSum += static_cast<double>(countSurfaceCtrlPts(patch.surface));
		++ctrlPatchCount;
	}

	if (report)
	{
		report->multiResolutionReducedCount = reducedCount;
		report->avgCtrlPtsPerPatch = ctrlPatchCount > 0 ? ctrlSum / static_cast<double>(ctrlPatchCount) : 0.0;
		report->totalCtrlPts = static_cast<int>(ctrlSum);
	}

	(void)errMsg;
	RunLogger::info(std::string("multi-resolution fit: reduced ") + std::to_string(reducedCount) + " patches");
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
