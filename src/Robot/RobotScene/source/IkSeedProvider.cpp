/// @file IkSeedProvider.cpp
/// @brief 轴配置感知种子扩展

#include "IkSeedProvider.h"

#include "IkSeedExpand.h"

#include <algorithm>
#include <cctype>
#include <cmath>

namespace RobotInstruction
{
int ikJointIndexByNameHint(const std::vector<std::string>& jointNames, const char* hint, int fallbackIndex)
{
	if (hint)
	{
		for (size_t i = 0; i < jointNames.size(); ++i)
		{
			std::string lower = jointNames[i];
			std::transform(lower.begin(), lower.end(), lower.begin(),
						   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
			if (lower.find(hint) != std::string::npos)
			{
				return static_cast<int>(i);
			}
		}
	}
	if (fallbackIndex >= 0 && fallbackIndex < static_cast<int>(jointNames.size()))
	{
		return fallbackIndex;
	}
	return -1;
}

namespace
{
double jointVectorDistance(const std::vector<double>& a, const std::vector<double>& b)
{
	const size_t n = std::min(a.size(), b.size());
	double s = 0.0;
	for (size_t i = 0; i < n; ++i)
	{
		const double d = a[i] - b[i];
		s += d * d;
	}
	return std::sqrt(s);
}
} // namespace

std::vector<std::vector<double>> buildIkSeedVariants(const std::vector<double>& qSeed,
													 const std::vector<std::string>& jointNames,
													 const MotionAxisConfiguration* axisCfg, const bool sphericalWrist,
													 const int elbowJointIndexOverride,
													 const int wristJointIndexOverride)
{
	std::vector<std::vector<double>> seeds = UrdfRobotLoader::expandPoseIkSeeds(
		qSeed, jointNames, nullptr, sphericalWrist, elbowJointIndexOverride, wristJointIndexOverride);
	auto pushUnique = [&](std::vector<double> q)
	{
		for (const auto& existing : seeds)
		{
			if (jointVectorDistance(existing, q) < 1e-6)
			{
				return;
			}
		}
		seeds.push_back(std::move(q));
	};
	const int n = static_cast<int>(qSeed.size());
	int elbowIdx = elbowJointIndexOverride;
	if (elbowIdx < 0 || elbowIdx >= n)
	{
		elbowIdx = ikJointIndexByNameHint(jointNames, "elbow", 2);
	}
	int wristIdx = wristJointIndexOverride;
	if (wristIdx < 0 || wristIdx >= n)
	{
		wristIdx = ikJointIndexByNameHint(jointNames, "wrist", 4);
	}
	const int j1Idx = ikJointIndexByNameHint(jointNames, nullptr, 0);
	if (axisCfg && motionAxisConfigurationRequiresConstraint(*axisCfg))
	{
		JointConfigurationClass want{};
		axisCfg->resolveEffective(want);
		if (elbowIdx >= 0)
		{
			if (want.elbow == ElbowPosture::Up)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(elbowIdx)] = 1.2;
				pushUnique(std::move(q));
				q = qSeed;
				q[static_cast<size_t>(elbowIdx)] = 2.0;
				pushUnique(std::move(q));
			}
			else if (want.elbow == ElbowPosture::Down)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(elbowIdx)] = -1.2;
				pushUnique(std::move(q));
				q = qSeed;
				q[static_cast<size_t>(elbowIdx)] = -2.0;
				pushUnique(std::move(q));
			}
		}
		// 腕+π / 解析向扰动仅球形腕有意义
		if (sphericalWrist && wristIdx >= 0)
		{
			if (want.wrist == WristPosture::Flip)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(wristIdx)] += 3.14159265358979323846;
				pushUnique(std::move(q));
			}
			else if (want.wrist == WristPosture::NoFlip)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(wristIdx)] *= 0.25;
				pushUnique(std::move(q));
			}
		}
		if (j1Idx >= 0)
		{
			if (want.arm == ArmPosture::Front)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(j1Idx)] = 0.0;
				pushUnique(std::move(q));
			}
			else if (want.arm == ArmPosture::Back)
			{
				std::vector<double> q = qSeed;
				q[static_cast<size_t>(j1Idx)] = 3.14159265358979323846;
				pushUnique(std::move(q));
			}
		}
	}
	return seeds;
}

} // namespace RobotInstruction
