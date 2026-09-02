/// @file JointSpaceDijkstraPlanner.cpp
/// @brief 关节空间均匀网格 Dijkstra

#include "JointSpaceDijkstraPlanner.h"

#include <chrono>
#include <cmath>
#include <limits>
#include <queue>
#include <string>
#include <unordered_map>
#include <vector>

namespace robot_path
{
namespace detail
{
namespace
{

struct GridSpec
{
	std::vector<int> dims;
	std::vector<double> lowerRad;
	std::vector<double> upperRad;
	double stepRad = 0.05;

	std::vector<int> qToCell(const std::vector<double>& q) const
	{
		std::vector<int> cell(q.size());
		for (std::size_t i = 0; i < q.size(); ++i)
		{
			const double span = upperRad[i] - lowerRad[i];
			if (span <= 1e-9 || dims[static_cast<std::size_t>(i)] <= 1)
			{
				cell[i] = 0;
				continue;
			}
			const double t = (q[i] - lowerRad[i]) / span;
			int idx = static_cast<int>(std::lround(t * static_cast<double>(dims[static_cast<std::size_t>(i)] - 1)));
			idx = std::max(0, std::min(dims[static_cast<std::size_t>(i)] - 1, idx));
			cell[i] = idx;
		}
		return cell;
	}

	std::vector<double> cellToQ(const std::vector<int>& cell) const
	{
		std::vector<double> q(cell.size());
		for (std::size_t i = 0; i < cell.size(); ++i)
		{
			if (dims[static_cast<std::size_t>(i)] <= 1)
				q[i] = lowerRad[i];
			else
			{
				const double span = upperRad[i] - lowerRad[i];
				q[i] = lowerRad[i] + span * static_cast<double>(cell[static_cast<std::size_t>(i)])
					   / static_cast<double>(dims[static_cast<std::size_t>(i)] - 1);
			}
		}
		return q;
	}

	std::string cellKey(const std::vector<int>& cell) const
	{
		std::string key;
		key.reserve(cell.size() * 2);
		for (int v : cell)
		{
			key.push_back(static_cast<char>(v & 0xFF));
			key.push_back(static_cast<char>((v >> 8) & 0xFF));
		}
		return key;
	}

	bool build(const JointLimits& lim, const double requestedStepRad)
	{
		if (lim.lowerRad.empty() || lim.lowerRad.size() != lim.upperRad.size())
			return false;
		stepRad = std::max(0.02, requestedStepRad);
		lowerRad = lim.lowerRad;
		upperRad = lim.upperRad;
		dims.assign(lim.lowerRad.size(), 1);
		constexpr int kMaxBinsPerJoint = 48;
		for (std::size_t i = 0; i < lim.lowerRad.size(); ++i)
		{
			const double span = lim.upperRad[i] - lim.lowerRad[i];
			if (span <= 1e-9)
			{
				dims[i] = 1;
				continue;
			}
			double jointStep = stepRad;
			int bins = static_cast<int>(std::ceil(span / jointStep)) + 1;
			if (bins > kMaxBinsPerJoint)
			{
				jointStep = span / static_cast<double>(kMaxBinsPerJoint - 1);
				bins = kMaxBinsPerJoint;
			}
			dims[i] = std::max(1, bins);
		}
		return true;
	}
};

double distL2(const std::vector<double>& a, const std::vector<double>& b)
{
	double s = 0.0;
	for (std::size_t i = 0; i < a.size(); ++i)
	{
		const double d = a[i] - b[i];
		s += d * d;
	}
	return std::sqrt(s);
}

struct DijkstraNode
{
	std::vector<int> cell;
	std::vector<double> q;
	std::string parentKey;
	double cost = 0.0;
};

bool reconstructPath(const std::unordered_map<std::string, DijkstraNode>& nodes, const std::string& goalKey,
					 std::vector<std::vector<double>>& path)
{
	path.clear();
	auto it = nodes.find(goalKey);
	if (it == nodes.end())
		return false;
	std::string cur = goalKey;
	while (!cur.empty())
	{
		const auto nit = nodes.find(cur);
		if (nit == nodes.end())
			return false;
		path.push_back(nit->second.q);
		cur = nit->second.parentKey;
	}
	std::reverse(path.begin(), path.end());
	return path.size() >= 2;
}

} // namespace

bool planJointSpaceDijkstra(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& goalQ,
							PathResult& out)
{
	out = PathResult{};
	out.plannerName = "Dijkstra";

	if (req.startJointRad.size() != goalQ.size() || req.startJointRad.empty())
	{
		out.errMsg = "Dijkstra: joint dimension mismatch";
		return false;
	}

	GridSpec grid;
	if (!grid.build(lim, req.options.longestValidSegmentRad))
	{
		out.errMsg = "Dijkstra: failed to build grid";
		return false;
	}

	const std::vector<int> startCell = grid.qToCell(req.startJointRad);
	const std::vector<int> goalCell = grid.qToCell(goalQ);
	const std::string startKey = grid.cellKey(startCell);
	const std::string goalKey = grid.cellKey(goalCell);

	if (startKey == goalKey)
	{
		out.jointTrajectoryRad = {req.startJointRad, goalQ};
		out.ok = true;
		return true;
	}

	const auto t0 = std::chrono::steady_clock::now();
	const double budgetSec = std::max(0.5, req.options.planningTimeSec);
	constexpr std::size_t kMaxExpand = 250000;

	struct HeapItem
	{
		double cost = 0.0;
		std::string key;
		bool operator>(const HeapItem& o) const { return cost > o.cost; }
	};

	std::unordered_map<std::string, DijkstraNode> nodes;
	nodes.reserve(4096);
	std::priority_queue<HeapItem, std::vector<HeapItem>, std::greater<HeapItem>> open;

	{
		DijkstraNode start{};
		start.cell = startCell;
		start.q = grid.cellToQ(startCell);
		start.parentKey.clear();
		start.cost = 0.0;
		if (!isStateValid(req, lim, start.q))
		{
			out.errMsg = "Dijkstra: start grid cell invalid";
			return false;
		}
		nodes.emplace(startKey, std::move(start));
		open.push(HeapItem{0.0, startKey});
	}

	while (!open.empty())
	{
		const auto elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
		if (elapsed >= budgetSec || nodes.size() >= kMaxExpand)
			break;

		const HeapItem top = open.top();
		open.pop();
		const auto curIt = nodes.find(top.key);
		if (curIt == nodes.end() || top.cost > curIt->second.cost + 1e-9)
			continue;

		if (top.key == goalKey)
			break;

		const DijkstraNode& cur = curIt->second;
		for (std::size_t dim = 0; dim < cur.cell.size(); ++dim)
		{
			if (grid.dims[dim] <= 1)
				continue;
			for (int delta : {-1, 1})
			{
				std::vector<int> nextCell = cur.cell;
				nextCell[dim] += delta;
				if (nextCell[dim] < 0 || nextCell[dim] >= grid.dims[dim])
					continue;

				const std::string nextKey = grid.cellKey(nextCell);
				const std::vector<double> nextQ = grid.cellToQ(nextCell);
				if (!isStateValid(req, lim, nextQ))
					continue;
				if (!isSegmentValid(req, lim, cur.q, nextQ, req.options.longestValidSegmentRad))
					continue;

				const double edgeCost = distL2(cur.q, nextQ);
				const double nextCost = cur.cost + edgeCost;
				const auto exist = nodes.find(nextKey);
				if (exist != nodes.end() && nextCost >= exist->second.cost - 1e-9)
					continue;

				DijkstraNode next{};
				next.cell = std::move(nextCell);
				next.q = nextQ;
				next.parentKey = top.key;
				next.cost = nextCost;
				nodes[nextKey] = std::move(next);
				open.push(HeapItem{nextCost, nextKey});
			}
		}
	}

	if (nodes.find(goalKey) == nodes.end())
	{
		out.errMsg = "Dijkstra: no path within time/grid budget";
		return false;
	}

	std::vector<std::vector<double>> path;
	if (!reconstructPath(nodes, goalKey, path))
	{
		out.errMsg = "Dijkstra: path reconstruction failed";
		return false;
	}

	// 首尾对齐真实起终点，避免网格量化偏差
	path.front() = req.startJointRad;
	path.back() = goalQ;
	out.jointTrajectoryRad = std::move(path);
	out.ok = true;
	return true;
}

} // namespace detail
} // namespace robot_path
