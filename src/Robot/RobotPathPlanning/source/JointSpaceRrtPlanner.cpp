/// @file JointSpaceRrtPlanner.cpp
/// @brief RRTConnect / RRT* 关节空间采样规划

#include "JointSpaceRrtPlanner.h"

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <random>
#include <vector>

namespace robot_path
{
namespace detail
{
namespace
{

struct Node
{
	std::vector<double> q;
	int parent = -1;
	double cost = 0.0;
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

std::vector<double> steer(const std::vector<double>& from, const std::vector<double>& to, const double stepRad)
{
	const double d = distL2(from, to);
	if (d <= stepRad || d < 1e-12)
		return to;
	std::vector<double> out(from.size());
	const double scale = stepRad / d;
	for (std::size_t i = 0; i < from.size(); ++i)
		out[i] = from[i] + scale * (to[i] - from[i]);
	return out;
}

int nearestNode(const std::vector<Node>& nodes, const std::vector<double>& q)
{
	int best = 0;
	double bestD = std::numeric_limits<double>::infinity();
	for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
	{
		const double d = distL2(nodes[static_cast<std::size_t>(i)].q, q);
		if (d < bestD)
		{
			bestD = d;
			best = i;
		}
	}
	return best;
}

std::vector<int> nearNodes(const std::vector<Node>& nodes, const std::vector<double>& q, const double radius)
{
	std::vector<int> out;
	const double r2 = radius * radius;
	for (int i = 0; i < static_cast<int>(nodes.size()); ++i)
	{
		if (distL2(nodes[static_cast<std::size_t>(i)].q, q) <= r2)
			out.push_back(i);
	}
	return out;
}

bool extractPath(const std::vector<Node>& nodes, int goalIdx, std::vector<std::vector<double>>& path)
{
	path.clear();
	if (goalIdx < 0 || goalIdx >= static_cast<int>(nodes.size()))
		return false;
	int cur = goalIdx;
	while (cur >= 0)
	{
		path.push_back(nodes[static_cast<std::size_t>(cur)].q);
		cur = nodes[static_cast<std::size_t>(cur)].parent;
	}
	std::reverse(path.begin(), path.end());
	return path.size() >= 2;
}

std::vector<double> sampleRandom(const JointLimits& lim, std::mt19937& rng, const std::vector<double>* goalBias,
								 const double goalBiasProb)
{
	std::uniform_real_distribution<double> u01(0.0, 1.0);
	std::vector<double> q(lim.lowerRad.size());
	if (goalBias && u01(rng) < goalBiasProb)
		return *goalBias;
	for (std::size_t i = 0; i < q.size(); ++i)
	{
		std::uniform_real_distribution<double> u(lim.lowerRad[i], lim.upperRad[i]);
		q[i] = u(rng);
	}
	return q;
}

bool planRrtConnect(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& startQ,
					const std::vector<double>& goalQ, PathResult& out)
{
	const double step = std::max(0.01, req.options.longestValidSegmentRad * 2.0);
	const auto t0 = std::chrono::steady_clock::now();
	const double budgetSec = std::max(0.1, req.options.planningTimeSec);
	std::mt19937 rng(req.options.rngSeed);

	std::vector<Node> treeA;
	std::vector<Node> treeB;
	treeA.push_back(Node{startQ, -1, 0.0});
	treeB.push_back(Node{goalQ, -1, 0.0});

	if (!isStateValid(req, lim, startQ))
	{
		out.errMsg = "start state invalid: " + describeStateInvalid(req, lim, startQ);
		return false;
	}
	if (!isStateValid(req, lim, goalQ))
	{
		out.errMsg = "goal state invalid: " + describeStateInvalid(req, lim, goalQ);
		return false;
	}

	bool swapA = false;
	while (true)
	{
		const auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration<double>(now - t0).count() > budgetSec)
			break;

		std::vector<Node>& ta = swapA ? treeB : treeA;
		std::vector<Node>& tb = swapA ? treeA : treeB;

		const std::vector<double> rnd = sampleRandom(lim, rng, &goalQ, 0.05);
		const int nn = nearestNode(ta, rnd);
		const std::vector<double> qNew = steer(ta[static_cast<std::size_t>(nn)].q, rnd, step);
		if (!isSegmentValid(req, lim, ta[static_cast<std::size_t>(nn)].q, qNew, req.options.longestValidSegmentRad))
		{
			swapA = !swapA;
			continue;
		}
		Node node{qNew, nn, ta[static_cast<std::size_t>(nn)].cost + distL2(ta[static_cast<std::size_t>(nn)].q, qNew)};
		ta.push_back(node);
		const int newIdx = static_cast<int>(ta.size()) - 1;

		const int nearB = nearestNode(tb, qNew);
		const std::vector<double> qConnect = steer(tb[static_cast<std::size_t>(nearB)].q, qNew, step);
		if (distL2(qConnect, qNew) < 1e-6
			&& isSegmentValid(req, lim, tb[static_cast<std::size_t>(nearB)].q, qNew, req.options.longestValidSegmentRad))
		{
			std::vector<std::vector<double>> pathA;
			std::vector<std::vector<double>> pathB;
			if (!extractPath(ta, newIdx, pathA))
				break;
			if (!extractPath(tb, nearB, pathB))
				break;
			if (swapA)
				std::swap(pathA, pathB);
			out.jointTrajectoryRad = pathA;
			for (std::size_t i = pathB.size(); i > 1; --i)
				out.jointTrajectoryRad.push_back(pathB[i - 1]);
			out.ok = true;
			out.plannerName = "RRTConnect";
			return true;
		}
		swapA = !swapA;
	}

	out.errMsg = "RRTConnect: no path within time budget";
	return false;
}

bool planRrtStar(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& startQ,
				 const std::vector<double>& goalQ, PathResult& out)
{
	const double step = std::max(0.01, req.options.longestValidSegmentRad * 2.0);
	const auto t0 = std::chrono::steady_clock::now();
	const double budgetSec = std::max(0.1, req.options.planningTimeSec);
	std::mt19937 rng(req.options.rngSeed ^ 0x9e3779b9u);

	std::vector<Node> nodes;
	nodes.push_back(Node{startQ, -1, 0.0});
	if (!isStateValid(req, lim, startQ))
	{
		out.errMsg = "start state invalid: " + describeStateInvalid(req, lim, startQ);
		return false;
	}

	int bestGoal = -1;
	double bestGoalCost = std::numeric_limits<double>::infinity();

	while (true)
	{
		const auto now = std::chrono::steady_clock::now();
		if (std::chrono::duration<double>(now - t0).count() > budgetSec)
			break;

		const std::vector<double> rnd = sampleRandom(lim, rng, &goalQ, 0.08);
		const int nn = nearestNode(nodes, rnd);
		const std::vector<double> qNew = steer(nodes[static_cast<std::size_t>(nn)].q, rnd, step);
		if (!isSegmentValid(req, lim, nodes[static_cast<std::size_t>(nn)].q, qNew, req.options.longestValidSegmentRad))
			continue;

		const double gamma = 2.0 * std::pow(1.0 + 1.0 / static_cast<double>(lim.lowerRad.size()), 1.0 / lim.lowerRad.size());
		const double radius = std::min(step * 4.0, gamma * std::pow(std::log(static_cast<double>(nodes.size()) + 1.0)
																	  / static_cast<double>(nodes.size() + 1),
																  1.0 / lim.lowerRad.size()));
		const std::vector<int> nbs = nearNodes(nodes, qNew, radius);

		int bestParent = nn;
		double bestCost = nodes[static_cast<std::size_t>(nn)].cost + distL2(nodes[static_cast<std::size_t>(nn)].q, qNew);
		for (int nb : nbs)
		{
			const double c = nodes[static_cast<std::size_t>(nb)].cost + distL2(nodes[static_cast<std::size_t>(nb)].q, qNew);
			if (c < bestCost
				&& isSegmentValid(req, lim, nodes[static_cast<std::size_t>(nb)].q, qNew, req.options.longestValidSegmentRad))
			{
				bestCost = c;
				bestParent = nb;
			}
		}

		Node node{qNew, bestParent, bestCost};
		nodes.push_back(node);
		const int newIdx = static_cast<int>(nodes.size()) - 1;

		for (int nb : nbs)
		{
			if (nb == bestParent)
				continue;
			const double c = nodes[static_cast<std::size_t>(newIdx)].cost
							 + distL2(nodes[static_cast<std::size_t>(newIdx)].q, nodes[static_cast<std::size_t>(nb)].q);
			if (c < nodes[static_cast<std::size_t>(nb)].cost
				&& isSegmentValid(req, lim, nodes[static_cast<std::size_t>(newIdx)].q, nodes[static_cast<std::size_t>(nb)].q,
								  req.options.longestValidSegmentRad))
			{
				nodes[static_cast<std::size_t>(nb)].parent = newIdx;
				nodes[static_cast<std::size_t>(nb)].cost = c;
			}
		}

		if (distL2(qNew, goalQ) < step
			&& isSegmentValid(req, lim, qNew, goalQ, req.options.longestValidSegmentRad))
		{
			const double gc = nodes[static_cast<std::size_t>(newIdx)].cost + distL2(qNew, goalQ);
			if (gc < bestGoalCost)
			{
				bestGoalCost = gc;
				nodes.push_back(Node{goalQ, newIdx, gc});
				bestGoal = static_cast<int>(nodes.size()) - 1;
			}
		}
	}

	if (bestGoal >= 0 && extractPath(nodes, bestGoal, out.jointTrajectoryRad))
	{
		out.ok = true;
		out.plannerName = "RRTstar";
		return true;
	}
	out.errMsg = "RRTstar: no path within time budget";
	return false;
}

} // namespace

bool planJointSpaceRrt(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& goalQ, PathResult& out)
{
	const std::vector<double>& startQ = req.startJointRad;
	if (startQ.size() != lim.lowerRad.size() || goalQ.size() != lim.lowerRad.size())
	{
		out.errMsg = "joint dimension mismatch";
		return false;
	}
	if (req.options.plannerId == "RRTstar" || req.options.plannerId == "RRT*")
		return planRrtStar(req, lim, startQ, goalQ, out);
	return planRrtConnect(req, lim, startQ, goalQ, out);
}

} // namespace detail
} // namespace robot_path
