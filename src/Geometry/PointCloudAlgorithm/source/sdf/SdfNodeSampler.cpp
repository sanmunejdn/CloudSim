#include "sdf/SdfNodeSampler.h"

#include "KdTreePointSet.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>

namespace pclalgo
{
namespace sdf
{
namespace
{

void normalizeSkin(std::vector<SkinWeight>& skin)
{
	double sumW = 0.0;
	for (const SkinWeight& sw : skin)
	{
		sumW += sw.w;
	}
	if (sumW <= 1e-12)
	{
		return;
	}
	for (SkinWeight& sw : skin)
	{
		sw.w /= sumW;
	}
}

/// 沿网格邻接找最近 k 个变形节点；搜索半径只作权重尺度，收齐 k 个前不硬截断
void skinByMeshBfs(const std::vector<float>& xyz, const std::vector<std::vector<int>>& vertAdj,
				   const std::vector<int>& vertToNode, std::size_t vi, double weightRadius, int kSkin,
				   std::vector<SkinWeight>& outSkin)
{
	outSkin.clear();
	const std::size_t n = vertAdj.size();
	if (vi >= n || kSkin <= 0)
	{
		return;
	}
	const double searchLimit = std::max(weightRadius * 6.0, weightRadius + 1e-6);
	constexpr int kMaxHops = 96;

	std::vector<double> dist(n, std::numeric_limits<double>::max());
	std::vector<int> hops(n, -1);
	std::queue<std::size_t> q;
	dist[vi] = 0.0;
	hops[vi] = 0;
	q.push(vi);
	std::vector<std::pair<double, int>> hits;
	hits.reserve(static_cast<std::size_t>(kSkin) * 4U);

	while (!q.empty())
	{
		const std::size_t u = q.front();
		q.pop();
		const double du = dist[u];
		const int hu = hops[u];
		if (du > searchLimit || hu > kMaxHops)
		{
			continue;
		}
		const int nodeId = vertToNode[u];
		if (nodeId >= 0)
		{
			hits.push_back({du, nodeId});
		}
		// 已够 k 个且队列前沿更远则停
		if (static_cast<int>(hits.size()) >= kSkin * 3 && du > weightRadius * 2.0)
		{
			// 仍继续一小段以稳定第 k 邻距离；由 searchLimit 收束
		}
		if (hu >= kMaxHops)
		{
			continue;
		}
		const double ux = xyz[u * 3U];
		const double uy = xyz[u * 3U + 1U];
		const double uz = xyz[u * 3U + 2U];
		for (int nb : vertAdj[u])
		{
			if (nb < 0)
			{
				continue;
			}
			const std::size_t v = static_cast<std::size_t>(nb);
			const double dx = xyz[v * 3U] - ux;
			const double dy = xyz[v * 3U + 1U] - uy;
			const double dz = xyz[v * 3U + 2U] - uz;
			const double nd = du + std::sqrt(dx * dx + dy * dy + dz * dz);
			if (nd < dist[v] && nd <= searchLimit)
			{
				dist[v] = nd;
				hops[v] = hu + 1;
				q.push(v);
			}
		}
	}

	std::sort(hits.begin(), hits.end(),
			  [](const std::pair<double, int>& a, const std::pair<double, int>& b) { return a.first < b.first; });
	std::vector<char> seenNode;
	int maxNode = -1;
	for (const auto& h : hits)
	{
		maxNode = std::max(maxNode, h.second);
	}
	if (maxNode >= 0)
	{
		seenNode.assign(static_cast<std::size_t>(maxNode) + 1U, 0);
	}
	std::vector<std::pair<double, int>> uniqueHits;
	uniqueHits.reserve(static_cast<std::size_t>(kSkin));
	for (const auto& h : hits)
	{
		if (h.second < 0 || seenNode[static_cast<std::size_t>(h.second)])
		{
			continue;
		}
		seenNode[static_cast<std::size_t>(h.second)] = 1;
		uniqueHits.push_back(h);
		if (static_cast<int>(uniqueHits.size()) >= kSkin)
		{
			break;
		}
	}
	if (uniqueHits.empty())
	{
		return;
	}
	// 权重半径取「第 k 邻距离」与给定 radius 的较大者，避免几乎全是单节点
	const double rEff = std::max(weightRadius, uniqueHits.back().first * 1.05 + 1e-12);
	for (const auto& h : uniqueHits)
	{
		const double t = std::min(1.0, h.first / rEff);
		const double u = 1.0 - t * t;
		const double w = u * u * u;
		if (w > 1e-12)
		{
			outSkin.push_back(SkinWeight{h.second, w});
		}
	}
	if (outSkin.empty())
	{
		outSkin.push_back(SkinWeight{uniqueHits.front().second, 1.0});
	}
	normalizeSkin(outSkin);
}

} // namespace

bool buildDeformGraph(const std::vector<float>& xyz, double sampleRadiusMm, int kSkin, int kNodeNeighbors,
					  DeformGraph& out, std::string* errMsg, const std::vector<std::vector<int>>* vertAdj)
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

	std::vector<char> taken(n, 0);
	std::vector<std::size_t> nodes;
	nodes.reserve(std::min<std::size_t>(n, 8192U));
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
		if (nodes.size() >= 8192U)
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
	std::vector<int> vertToNode(n, -1);
	for (std::size_t j = 0; j < nodes.size(); ++j)
	{
		const std::size_t i = nodes[j];
		out.nodeRest[j] = Eigen::Vector3d(xyz[i * 3U], xyz[i * 3U + 1U], xyz[i * 3U + 2U]);
		nodeXyz[j * 3U] = xyz[i * 3U];
		nodeXyz[j * 3U + 1U] = xyz[i * 3U + 1U];
		nodeXyz[j * 3U + 2U] = xyz[i * 3U + 2U];
		vertToNode[i] = static_cast<int>(j);
	}

	const bool useMesh = vertAdj != nullptr && vertAdj->size() == n;
	out.nodeNeighbors.resize(nodes.size());

	if (useMesh)
	{
		KdTreePointSet nodeTree(nodeXyz);
		const double influenceR = sampleRadiusMm * 4.0;
		const double searchR = influenceR * 1.5;
		constexpr int kMaxHops = 64;

		// 从节点向外扩散：O(节点数×邻域)，避免对每个顶点整图 BFS
		std::vector<std::vector<std::pair<double, int>>> cand(n);
		for (std::size_t j = 0; j < nodes.size(); ++j)
		{
			const std::size_t seed = nodes[j];
			std::vector<double> dist(n, std::numeric_limits<double>::max());
			std::vector<int> hops(n, -1);
			std::queue<std::size_t> q;
			dist[seed] = 0.0;
			hops[seed] = 0;
			q.push(seed);
			while (!q.empty())
			{
				const std::size_t u = q.front();
				q.pop();
				const double du = dist[u];
				const int hu = hops[u];
				if (du > searchR || hu > kMaxHops)
				{
					continue;
				}
				cand[u].push_back({du, static_cast<int>(j)});
				if (hu >= kMaxHops)
				{
					continue;
				}
				const double ux = xyz[u * 3U];
				const double uy = xyz[u * 3U + 1U];
				const double uz = xyz[u * 3U + 2U];
				for (int nb : (*vertAdj)[u])
				{
					if (nb < 0)
					{
						continue;
					}
					const std::size_t v = static_cast<std::size_t>(nb);
					const double dx = xyz[v * 3U] - ux;
					const double dy = xyz[v * 3U + 1U] - uy;
					const double dz = xyz[v * 3U + 2U] - uz;
					const double nd = du + std::sqrt(dx * dx + dy * dy + dz * dz);
					if (nd < dist[v] && nd <= searchR)
					{
						dist[v] = nd;
						hops[v] = hu + 1;
						q.push(v);
					}
				}
			}
		}

		out.nodeNeighbors.resize(nodes.size());
		for (std::size_t j = 0; j < nodes.size(); ++j)
		{
			std::vector<SkinWeight> tmp;
			skinByMeshBfs(xyz, *vertAdj, vertToNode, nodes[j], influenceR, kNodeNeighbors + 1, tmp);
			for (const SkinWeight& sw : tmp)
			{
				if (sw.node == static_cast<int>(j))
				{
					continue;
				}
				out.nodeNeighbors[j].push_back(sw.node);
				if (static_cast<int>(out.nodeNeighbors[j].size()) >= kNodeNeighbors)
				{
					break;
				}
			}
			if (static_cast<int>(out.nodeNeighbors[j].size()) < kNodeNeighbors)
			{
				std::vector<std::size_t> idx;
				std::vector<double> d2;
				nodeTree.findKNearest(nodeXyz[j * 3U], nodeXyz[j * 3U + 1U], nodeXyz[j * 3U + 2U],
									  static_cast<unsigned int>(kNodeNeighbors + 1), idx, d2);
				std::vector<char> have(nodes.size(), 0);
				have[j] = 1;
				for (int nb : out.nodeNeighbors[j])
				{
					if (nb >= 0 && static_cast<std::size_t>(nb) < have.size())
					{
						have[static_cast<std::size_t>(nb)] = 1;
					}
				}
				for (std::size_t k = 0; k < idx.size(); ++k)
				{
					if (idx[k] >= nodes.size() || have[idx[k]])
					{
						continue;
					}
					out.nodeNeighbors[j].push_back(static_cast<int>(idx[k]));
					have[idx[k]] = 1;
					if (static_cast<int>(out.nodeNeighbors[j].size()) >= kNodeNeighbors)
					{
						break;
					}
				}
			}
		}

		out.vertSkin.resize(n);
		for (std::size_t i = 0; i < n; ++i)
		{
			auto& list = cand[i];
			std::sort(list.begin(), list.end(),
					  [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
						  return a.first < b.first;
					  });
			// 同一节点可能被多次写入，去重
			std::vector<std::pair<double, int>> unique;
			unique.reserve(static_cast<std::size_t>(kSkin));
			std::vector<char> seen(nodes.size(), 0);
			for (const auto& h : list)
			{
				if (h.second < 0 || static_cast<std::size_t>(h.second) >= seen.size() || seen[static_cast<std::size_t>(h.second)])
				{
					continue;
				}
				seen[static_cast<std::size_t>(h.second)] = 1;
				unique.push_back(h);
				if (static_cast<int>(unique.size()) >= kSkin)
				{
					break;
				}
			}
			out.vertSkin[i].clear();
			if (unique.empty())
			{
				std::vector<std::size_t> idx;
				std::vector<double> d2;
				nodeTree.findKNearest(xyz[i * 3U], xyz[i * 3U + 1U], xyz[i * 3U + 2U],
									  static_cast<unsigned int>(kSkin), idx, d2);
				const double rEff = (!d2.empty()) ? std::max(influenceR, std::sqrt(d2.back()) * 1.05) : influenceR;
				for (std::size_t k = 0; k < idx.size(); ++k)
				{
					const double d = std::sqrt(std::max(0.0, d2[k]));
					const double t = std::min(1.0, d / rEff);
					const double u = 1.0 - t * t;
					out.vertSkin[i].push_back(SkinWeight{static_cast<int>(idx[k]), u * u * u});
				}
			}
			else
			{
				const double rEff = std::max(influenceR, unique.back().first * 1.05 + 1e-12);
				for (const auto& h : unique)
				{
					const double t = std::min(1.0, h.first / rEff);
					const double u = 1.0 - t * t;
					out.vertSkin[i].push_back(SkinWeight{h.second, u * u * u});
				}
			}
			normalizeSkin(out.vertSkin[i]);
		}
		return true;
	}

	KdTreePointSet nodeTree(nodeXyz);
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
	const double rSkin = std::max(1e-6, sampleRadiusMm * 2.5);
	for (std::size_t i = 0; i < n; ++i)
	{
		std::vector<std::size_t> idx;
		std::vector<double> d2;
		nodeTree.findKNearest(xyz[i * 3U], xyz[i * 3U + 1U], xyz[i * 3U + 2U], static_cast<unsigned int>(kSkin), idx,
							  d2);
		out.vertSkin[i].clear();
		const double rEff = (!d2.empty()) ? std::max(rSkin, std::sqrt(d2.back()) * 1.05) : rSkin;
		for (std::size_t k = 0; k < idx.size(); ++k)
		{
			const double d = std::sqrt(std::max(0.0, d2[k]));
			const double t = std::min(1.0, d / rEff);
			const double u = 1.0 - t * t;
			out.vertSkin[i].push_back(SkinWeight{static_cast<int>(idx[k]), u * u * u});
		}
		if (out.vertSkin[i].empty() && !idx.empty())
		{
			out.vertSkin[i].push_back(SkinWeight{static_cast<int>(idx[0]), 1.0});
		}
		normalizeSkin(out.vertSkin[i]);
	}
	return true;
}

} // namespace sdf
} // namespace pclalgo
