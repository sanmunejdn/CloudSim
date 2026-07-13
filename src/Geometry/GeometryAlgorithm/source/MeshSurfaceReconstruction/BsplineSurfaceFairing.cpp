#include "MeshSurfaceReconstructionInternal.h"

#include "detail/OccIncludes.h"

#include <TColgp_Array2OfPnt.hxx>

#include <algorithm>
#include <cmath>
#include <vector>

namespace geoalgo
{
namespace meshrecon
{
namespace
{

double thirdDerivJumpU(const TColgp_Array2OfPnt& poles, const int iu, const int iv)
{
	if (iu < poles.LowerRow() + 1 || iu > poles.UpperRow() - 2)
	{
		return 0.0;
	}
	const gp_Pnt p0 = poles.Value(iu - 1, iv);
	const gp_Pnt p1 = poles.Value(iu, iv);
	const gp_Pnt p2 = poles.Value(iu + 1, iv);
	const gp_Pnt p3 = poles.Value(iu + 2, iv);
	const double dx = (p3.X() - 3.0 * p2.X() + 3.0 * p1.X() - p0.X());
	const double dy = (p3.Y() - 3.0 * p2.Y() + 3.0 * p1.Y() - p0.Y());
	const double dz = (p3.Z() - 3.0 * p2.Z() + 3.0 * p1.Z() - p0.Z());
	return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double globalFairingMetric(const TColgp_Array2OfPnt& poles)
{
	double sum = 0.0;
	for (int iu = poles.LowerRow() + 1; iu <= poles.UpperRow() - 2; ++iu)
	{
		for (int iv = poles.LowerCol() + 1; iv <= poles.UpperCol() - 2; ++iv)
		{
			const double jump = thirdDerivJumpU(poles, iu, iv);
			if (std::isfinite(jump))
			{
				sum += jump;
			}
		}
	}
	return sum;
}

bool fairSingleSurface(Handle(Geom_BSplineSurface)& surface, const MeshSurfaceReconstructParams& params)
{
	if (surface.IsNull())
	{
		return false;
	}
	TColgp_Array2OfPnt poles = surface->Poles();
	TColgp_Array2OfPnt backup = poles;
	const int lr = poles.LowerRow();
	const int ur = poles.UpperRow();
	const int lc = poles.LowerCol();
	const int uc = poles.UpperCol();
	const int borderGuard = params.fairingProtectBoundaries ? 1 : 0;
	double metric = globalFairingMetric(poles);
	if (!std::isfinite(metric) || metric < params.fairingEpsilon)
	{
		return true;
	}

	int failCount = 0;
	const int maxFail = std::max(
		1,
		(ur - lr - 2 - 2 * borderGuard) * (uc - lc - 2 - 2 * borderGuard));
	for (int iter = 0; iter < params.fairingMaxIterations; ++iter)
	{
		double maxJump = 0.0;
		int bi = lr + 1 + borderGuard;
		int bj = lc + 1 + borderGuard;
		for (int iu = lr + 1 + borderGuard; iu <= ur - 2 - borderGuard; ++iu)
		{
			for (int iv = lc + 1 + borderGuard; iv <= uc - 2 - borderGuard; ++iv)
			{
				const double z = thirdDerivJumpU(poles, iu, iv);
				if (z > maxJump)
				{
					maxJump = z;
					bi = iu;
					bj = iv;
				}
			}
		}
		if (maxJump < params.fairingEpsilon)
		{
			break;
		}

		for (int di = -1; di <= 1; ++di)
		{
			for (int dj = -1; dj <= 1; ++dj)
			{
				const int ii = bi + di;
				const int jj = bj + dj;
				if (ii < lr + borderGuard || jj < lc + borderGuard || ii > ur - borderGuard
					|| jj > uc - borderGuard)
				{
					continue;
				}
				const gp_Pnt p = poles.Value(ii, jj);
				const gp_Pnt n = poles.Value(bi, bj);
				poles.SetValue(
					ii,
					jj,
					gp_Pnt(
						p.X() + 0.15 * (n.X() - p.X()),
						p.Y() + 0.15 * (n.Y() - p.Y()),
						p.Z() + 0.15 * (n.Z() - p.Z())));
			}
		}

		const double newMetric = globalFairingMetric(poles);
		if (std::isfinite(newMetric) && newMetric < metric)
		{
			metric = newMetric;
			backup = poles;
			failCount = 0;
		}
		else
		{
			poles = backup;
			++failCount;
			if (failCount >= maxFail)
			{
				break;
			}
		}
	}
	Handle(Geom_BSplineSurface) rebuilt;
	if (tryRebuildBsplineSurface(surface, poles, rebuilt))
	{
		surface = rebuilt;
	}
	else
	{
		(void)tryRebuildBsplineSurface(surface, backup, surface);
	}
	return true;
}

} // namespace

bool fairBsplinePatches(
	std::vector<QuadPatch>& patches,
	const MeshSurfaceReconstructParams& params,
	double& outGlobalMetric,
	std::string* errMsg)
{
	outGlobalMetric = 0.0;
	for (QuadPatch& patch : patches)
	{
		if (patch.surface.IsNull())
		{
			continue;
		}
		if (!fairSingleSurface(patch.surface, params))
		{
			if (errMsg)
			{
				*errMsg = "surface fairing failed";
			}
			return false;
		}
		const double m = globalFairingMetric(patch.surface->Poles());
		if (std::isfinite(m))
		{
			outGlobalMetric += m;
		}
	}
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
