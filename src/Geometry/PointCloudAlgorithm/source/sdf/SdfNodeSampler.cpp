#include "sdf/SdfNodeSampler.h"

#include "KdTreePointSet.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace pclalgo
{
namespace sdf
{

bool buildDeformGraph(const std::vector<float>& xyz, double sampleRadiusMm, int kSkin, int kNodeNeighbors,
					  DeformGraph& out, std::string* errMsg)
{
	out = DeformGraph{};
	const std::size_t n = xyz.size() / 3U;
	if (n < 3U)
	{
		if (errMsg)
		{
			*errMsg = "SdfNodeSampler: too few points";
		}
		return false;
	}
	kSkin = std::max(1, kSkin);
	kNodeNeighbors = std::max(1, kNodeNeighbors);
	const double r2 = std::max(1e-6, sampleRadiusMm) * std::max(1e-6, sampleRadiusMm);

	// FPS：按半径抑制
	std::vector<char> taken(n, 0);
	std::vector<std::size_t> nodes;
	nodes.reserve(std::min<std::size_t>(n, 2048U));
	nodes.push_back(0);
	taken[0] = 1;
	std::vector<double> minDist2(n, std::numeric_limits<double>::max());
	auto updateDist = [&](std::size_t seed) {
		const double sx = xyz[seed * 3U];
		const double sy = xyz[seed * 3U + 1U];
		const double sz = xyz[seed * 3U + 2U];
		for (std::size_t i = 0; i < n; ++i)
		{
			if (taken[i])
			{
				continue;
			}
			const double dx = xyz[i * 3U] - sx;
			const double dy = xyz[i * 3U + 1U] - sy;
			const double dz = xyz[i * 3U + 2U] - sz;
			minDist2[i] = std::min(minDist2[i], dx * dx + dy * dy + dz * dz);
		}
	};
	updateDist(0);
	while (true)
	{
		std::size_t best = static_cast<std::size_t>(-1);
		double bestD = -1.0;
		for (std::size_t i = 0; i < n; ++i)
		{
			if (taken[i])
			{
				continue;
			}
			if (minDist2[i] > bestD)
			{
				bestD = minDist2[i];
				best = i;
			}
		}
		if (best == static_cast<std::size_t>(-1) || bestD < r2)
		{
			break;
		}
		nodes.push_back(best);
		taken[best] = 1;
		updateDist(best);
		if (nodes.size() >= 2048U)
		{
			break;
		}
	}
	if (nodes.empty())
	{
		if (errMsg)
		{
			*errMsg = "SdfNodeSampler: no nodes";
		}
		return false;
	}

	out.nodeRest.resize(nodes.size());
	std::vector<float> nodeXyz(nodes.size() * 3U);
	for (std::size_t j = 0; j < nodes.size(); ++j)
	{
		const std::size_t i = nodes[j];
		out.nodeRest[j] = Eigen::Vector3d(xyz[i * 3U], xyz[i * 3U + 1U], xyz[i * 3U + 2U]);
		nodeXyz[j * 3U] = xyz[i * 3U];
		nodeXyz[j * 3U + 1U] = xyz[i * 3U + 1U];
		nodeXyz[j * 3U + 2U] = xyz[i * 3U + 2U];
	}

	KdTreePointSet nodeTree(nodeXyz);
	out.nodeNeighbors.resize(nodes.size());
	for (std::size_t j = 0; j < nodes.size(); ++j)
	{
		std::vector<std::size_t> idx;
		std::vector<double> d2;
		nodeTree.findKNearest(nodeXyz[j * 3U], nodeXyz[j * 3U + 1U], nodeXyz[j * 3U + 2U],
							  static_cast<unsigned int>(kNodeNeighbors + 1), idx, d2);
		for (std::size_t k = 0; k < idx.size(); ++k)
		{
			if (idx[k] == j)
			{
				continue;
			}
			out.nodeNeighbors[j].push_back(static_cast<int>(idx[k]));
		}
	}

	out.vertSkin.resize(n);
	for (std::size_t i = 0; i < n; ++i)
	{
		std::vector<std::size_t> idx;
		std::vector<double> d2;
		nodeTree.findKNearest(xyz[i * 3U], xyz[i * 3U + 1U], xyz[i * 3U + 2U], static_cast<unsigned int>(kSkin), idx,
							  d2);
		double sumW = 0.0;
		out.vertSkin[i].clear();
		for (std::size_t k = 0; k < idx.size(); ++k)
		{
			const double w = 1.0 / std::max(1e-9, d2[k]);
			out.vertSkin[i].push_back(SkinWeight{static_cast<int>(idx[k]), w});
			sumW += w;
		}
		if (sumW > 0.0)
		{
			for (SkinWeight& sw : out.vertSkin[i])
			{
				sw.w /= sumW;
			}
		}
	}
	return true;
}

} // namespace sdf
} // namespace pclalgo
