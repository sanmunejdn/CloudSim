/// @file RuckigPtpTrajectory.cpp
/// @brief PTP Ruckig 轨迹（本文件须 C++20）

#include "RuckigPtpTrajectory.h"

#include <algorithm>
#include <cmath>

#include <ruckig/ruckig.hpp>

namespace RobotInstruction
{
namespace
{
constexpr double kDefaultMaxVel = 1.5;
constexpr double kDefaultMaxAcc = 4.0;
constexpr double kDefaultMaxJerk = 20.0;
constexpr double kDegToRad = 0.017453292519943295;
} // namespace

RuckigPtpLimits ruckigLimitsFromPtpDegScalars(const size_t dof, const double speedDegPerSec,
											  const double accelDegPerSec2)
{
	RuckigPtpLimits lim;
	const double vmax = std::max(1e-6, speedDegPerSec * kDegToRad);
	const double amax = std::max(1e-6, accelDegPerSec2 * kDegToRad);
	lim.maxVelocityRadPerSec.assign(dof, vmax);
	lim.maxAccelerationRadPerSec2.assign(dof, amax);
	lim.maxJerkRadPerSec3.assign(dof, std::max(10.0, amax * 5.0));
	return lim;
}

RuckigPtpLimits mapMotionScalarToRuckigLimits(const size_t dof, const double speed, const double accel)
{
	// 兼容旧调用：按 °/s 解读，与 PtpPlanner / 梯形一致
	return ruckigLimitsFromPtpDegScalars(dof, speed, accel);
}

bool buildRuckigPtpJointTrajectory(const std::vector<double>& q0, const std::vector<double>& q1,
								   const RuckigPtpLimits& limits, double sampleDtSec,
								   std::vector<std::vector<double>>& outTrajectoryRad, double& outDurationSec)
{
	outTrajectoryRad.clear();
	outDurationSec = 0.0;
	if (q0.empty() || q0.size() != q1.size())
	{
		return false;
	}
	const size_t dof = q0.size();
	if (sampleDtSec < 1e-4)
	{
		sampleDtSec = 0.02;
	}

	// 相对起点折到最近圈，避免 Ruckig 走 ±2π 长弧
	std::vector<double> q1Near = q1;
	constexpr double kTwoPi = 6.283185307179586;
	for (size_t i = 0; i < dof; ++i)
	{
		double d = q1Near[i] - q0[i];
		d -= kTwoPi * std::round(d / kTwoPi);
		q1Near[i] = q0[i] + d;
	}

	ruckig::Ruckig<ruckig::DynamicDOFs> otg(dof);
	ruckig::InputParameter<ruckig::DynamicDOFs> input(dof);
	ruckig::Trajectory<ruckig::DynamicDOFs> trajectory(dof);

	input.current_position = q0;
	input.current_velocity = std::vector<double>(dof, 0.0);
	input.current_acceleration = std::vector<double>(dof, 0.0);
	input.target_position = q1Near;
	input.target_velocity = std::vector<double>(dof, 0.0);
	input.target_acceleration = std::vector<double>(dof, 0.0);

	auto fillOrDefault = [&](std::vector<double>& dst, const std::vector<double>& src, double def)
	{
		dst.resize(dof);
		for (size_t i = 0; i < dof; ++i)
		{
			dst[i] = (src.size() == dof && src[i] > 1e-9) ? src[i] : def;
		}
	};
	fillOrDefault(input.max_velocity, limits.maxVelocityRadPerSec, kDefaultMaxVel);
	fillOrDefault(input.max_acceleration, limits.maxAccelerationRadPerSec2, kDefaultMaxAcc);
	fillOrDefault(input.max_jerk, limits.maxJerkRadPerSec3, kDefaultMaxJerk);

	const ruckig::Result result = otg.calculate(input, trajectory);
	if (result != ruckig::Result::Finished && result != ruckig::Result::Working)
	{
		return false;
	}
	outDurationSec = trajectory.get_duration();
	if (outDurationSec < 1e-6)
	{
		outTrajectoryRad.push_back(q1Near);
		outDurationSec = 0.05;
		return true;
	}

	const size_t nSamples = static_cast<size_t>(std::ceil(outDurationSec / sampleDtSec)) + 1;
	outTrajectoryRad.reserve(nSamples);
	std::vector<double> newPos(dof), newVel(dof), newAcc(dof);
	for (size_t i = 0; i < nSamples; ++i)
	{
		const double t = std::min(outDurationSec, static_cast<double>(i) * sampleDtSec);
		trajectory.at_time(t, newPos, newVel, newAcc);
		outTrajectoryRad.push_back(newPos);
	}
	if (outTrajectoryRad.empty())
	{
		outTrajectoryRad.push_back(q1Near);
	}
	else
	{
		bool same = outTrajectoryRad.back().size() == q1Near.size();
		if (same)
		{
			for (size_t i = 0; i < q1Near.size(); ++i)
			{
				if (std::abs(outTrajectoryRad.back()[i] - q1Near[i]) > 1e-9)
				{
					same = false;
					break;
				}
			}
		}
		if (!same)
		{
			outTrajectoryRad.push_back(q1Near);
		}
	}
	return true;
}

} // namespace RobotInstruction
