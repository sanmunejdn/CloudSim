/// @file IkSeedExpand.cpp
/// @brief 位姿 IK 多种子与折圈选解

#include "IkSeedExpand.h"

#include <cmath>

namespace UrdfRobotLoader
{
namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;

void pushUnique(std::vector<std::vector<double>>& seeds, std::vector<double> q)
{
	for (const auto& existing : seeds)
	{
		if (existing.size() != q.size())
		{
			continue;
		}
		double d2 = 0.0;
		for (size_t i = 0; i < q.size(); ++i)
		{
			const double d = existing[i] - q[i];
			d2 += d * d;
		}
		if (d2 < 1e-12)
		{
			return;
		}
	}
	seeds.push_back(std::move(q));
}

double wrapToPi(double a)
{
	a = std::fmod(a + kPi, kTwoPi);
	if (a < 0.0)
	{
		a += kTwoPi;
	}
	return a - kPi;
}
} // namespace

std::vector<std::vector<double>> expandPoseIkSeeds(const std::vector<double>& primarySeed,
												   const std::vector<std::vector<double>>* extraSeeds)
{
	std::vector<std::vector<double>> seeds;
	if (primarySeed.empty())
	{
		return seeds;
	}
	pushUnique(seeds, primarySeed);
	if (extraSeeds)
	{
		for (const auto& e : *extraSeeds)
		{
			if (e.size() == primarySeed.size())
			{
				pushUnique(seeds, e);
			}
		}
	}

	const int n = static_cast<int>(primarySeed.size());
	const int elbowIdx = n >= 3 ? 2 : -1;
	const int wristIdx = n >= 5 ? 4 : (n >= 2 ? n - 2 : -1);
	const int j5Idx = n >= 2 ? n - 2 : -1;
	const int tailIdx = n - 1;

	if (elbowIdx >= 0)
	{
		std::vector<double> qElbow = primarySeed;
		qElbow[static_cast<size_t>(elbowIdx)] = -qElbow[static_cast<size_t>(elbowIdx)];
		pushUnique(seeds, std::move(qElbow));
	}
	if (wristIdx >= 0)
	{
		std::vector<double> qWrist = primarySeed;
		qWrist[static_cast<size_t>(wristIdx)] += kPi;
		pushUnique(seeds, std::move(qWrist));
		if (elbowIdx >= 0)
		{
			std::vector<double> qBoth = primarySeed;
			qBoth[static_cast<size_t>(elbowIdx)] = -qBoth[static_cast<size_t>(elbowIdx)];
			qBoth[static_cast<size_t>(wristIdx)] += kPi;
			pushUnique(seeds, std::move(qBoth));
		}
	}
	if (j5Idx >= 0)
	{
		std::vector<double> qFlip = primarySeed;
		qFlip[static_cast<size_t>(j5Idx)] = -qFlip[static_cast<size_t>(j5Idx)];
		pushUnique(seeds, std::move(qFlip));
	}
	if (tailIdx >= 0)
	{
		const double qTail0 = primarySeed[static_cast<size_t>(tailIdx)];
		const double kTailOffsets[] = {0.25 * kPi, -0.25 * kPi, 0.5 * kPi, -0.5 * kPi,
									   0.75 * kPi, -0.75 * kPi, kPi,	   -kPi};
		for (double off : kTailOffsets)
		{
			std::vector<double> qTail = primarySeed;
			qTail[static_cast<size_t>(tailIdx)] = qTail0 + off;
			pushUnique(seeds, std::move(qTail));
		}
		if (j5Idx >= 0)
		{
			for (double off : kTailOffsets)
			{
				std::vector<double> q = primarySeed;
				q[static_cast<size_t>(j5Idx)] = -q[static_cast<size_t>(j5Idx)];
				q[static_cast<size_t>(tailIdx)] = qTail0 + off;
				pushUnique(seeds, std::move(q));
			}
		}
	}
	return seeds;
}

double wrappedJointDistance(const std::vector<double>& q, const std::vector<double>& ref)
{
	if (q.size() != ref.size() || q.empty())
	{
		return 1e30;
	}
	double sum = 0.0;
	for (size_t i = 0; i < q.size(); ++i)
	{
		const double d = wrapToPi(q[i] - ref[i]);
		sum += d * d;
	}
	return std::sqrt(sum);
}

std::vector<double> selectNearestWrappedSolution(const std::vector<std::vector<double>>& candidates,
												 const std::vector<double>& seedRef)
{
	std::vector<double> best;
	double bestD = 1e30;
	for (const auto& c : candidates)
	{
		const double d = wrappedJointDistance(c, seedRef);
		if (d < bestD)
		{
			bestD = d;
			best = c;
		}
	}
	return best;
}

} // namespace UrdfRobotLoader
