/// @file JunctionBlend.cpp
/// @brief JunctionBlend 实现

#include "MeshSurfaceReconstructionInternal.h"
#include "detail/OccIncludes.h"

#include <cmath>
#include <unordered_map>
#include <vector>

#include <TColgp_Array2OfPnt.hxx>

namespace geoalgo
{
namespace meshrecon
{
namespace
{
double junctionWeight(const double u, const double v, const double w)
{
	const double s = (u + v + w) / 3.0;
	return s * s * (3.0 - 2.0 * s);
}

} // namespace

bool applyJunctionC2Blend(std::vector<QuadPatch>& patches, int junctionCount,
						  const MeshSurfaceReconstructParams& params, MeshSurfaceReconstructReport* report,
						  std::string* errMsg)
{
	(void)params;
	if (patches.size() < 3U)
	{
		return true;
	}

	std::unordered_map<int, std::vector<int>> adjacency;
	for (std::size_t i = 0; i < patches.size(); ++i)
	{
		for (const int nb : patches[i].neighborPatchIds)
		{
			adjacency[static_cast<int>(i)].push_back(nb);
		}
	}

	int blended = 0;
	double maxMove = 0.0;
	for (std::size_t i = 0; i < patches.size() && blended < junctionCount; ++i)
	{
		const auto it = adjacency.find(static_cast<int>(i));
		if (it == adjacency.end() || it->second.size() < 2)
		{
			continue;
		}
		std::vector<int> cluster = {static_cast<int>(i)};
		for (const int nb : it->second)
		{
			if (adjacency[nb].size() >= 2)
			{
				cluster.push_back(nb);
			}
		}
		if (cluster.size() < 3)
		{
			continue;
		}

		gp_Pnt avg{0, 0, 0};
		int count = 0;
		for (const int pid : cluster)
		{
			QuadPatch& patch = patches[static_cast<std::size_t>(pid)];
			if (patch.surface.IsNull())
			{
				continue;
			}
			TColgp_Array2OfPnt poles = patch.surface->Poles();
			const gp_Pnt corner = poles.Value(1, 1);
			avg = gp_Pnt(avg.X() + corner.X(), avg.Y() + corner.Y(), avg.Z() + corner.Z());
			++count;
		}
		if (count < 3)
		{
			continue;
		}
		avg = gp_Pnt(avg.X() / count, avg.Y() / count, avg.Z() / count);

		for (const int pid : cluster)
		{
			QuadPatch& patch = patches[static_cast<std::size_t>(pid)];
			if (patch.surface.IsNull())
			{
				continue;
			}
			TColgp_Array2OfPnt poles = patch.surface->Poles();
			const double wt = junctionWeight(1.0 / 3.0, 1.0 / 3.0, 1.0 / 3.0);
			const gp_Pnt c0 = poles.Value(1, 1);
			const gp_Pnt blend(c0.X() * (1.0 - wt) + avg.X() * wt, c0.Y() * (1.0 - wt) + avg.Y() * wt,
							   c0.Z() * (1.0 - wt) + avg.Z() * wt);
			const double d = c0.Distance(blend);
			if (d > maxMove)
			{
				maxMove = d;
			}
			poles.SetValue(1, 1, blend);
			(void)tryRebuildBsplineSurface(patch.surface, poles, patch.surface);
		}
		++blended;
	}

	if (report)
	{
		report->junctionBlendAppliedCount = blended;
		report->junctionBlendMaxMoveMm = maxMove;
	}

	(void)errMsg;
	return true;
}

} // namespace meshrecon
} // namespace geoalgo
