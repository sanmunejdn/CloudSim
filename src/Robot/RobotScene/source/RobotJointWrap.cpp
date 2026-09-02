/// @file RobotJointWrap.cpp
/// @brief 回转关节折圈

#include "RobotJointWrap.h"

#include <QString>
#include <algorithm>
#include <cmath>

namespace
{
void shiftJointTrajectoryByTargetDelta(std::vector<std::vector<double>>& traj,
									   const std::vector<double>& targetsBefore,
									   const std::vector<double>& targetsAfter)
{
	if (traj.empty() || targetsBefore.size() != targetsAfter.size() || targetsAfter.empty())
	{
		return;
	}
	const size_t n = targetsAfter.size();
	std::vector<double> delta(n, 0.0);
	bool any = false;
	for (size_t j = 0; j < n; ++j)
	{
		delta[j] = targetsAfter[j] - targetsBefore[j];
		if (std::abs(delta[j]) > 1e-12)
		{
			any = true;
		}
	}
	if (!any)
	{
		return;
	}
	for (std::vector<double>& sample : traj)
	{
		if (sample.size() != n)
		{
			continue;
		}
		for (size_t j = 0; j < n; ++j)
		{
			sample[j] += delta[j];
		}
	}
}

bool jointTrajectoryTailMatchesTargets(const std::vector<std::vector<double>>& traj,
									   const std::vector<double>& targets, const double eps = 1e-6)
{
	if (traj.empty() || traj.back().size() != targets.size())
	{
		return false;
	}
	const auto& back = traj.back();
	for (size_t j = 0; j < targets.size(); ++j)
	{
		if (std::abs(back[j] - targets[j]) > eps)
		{
			return false;
		}
	}
	return true;
}
} // namespace

bool normalizeJointRevolutionsToReference(std::vector<double>& q, const std::vector<double>& ref,
										  const QVector<double>& lowerRad, const QVector<double>& upperRad)
{
	const size_t n = q.size();
	if (ref.size() != n)
	{
		return false;
	}
	const bool hasLimits = lowerRad.size() == static_cast<int>(n) && upperRad.size() == static_cast<int>(n);
	if (!hasLimits)
	{
		return false;
	}
	constexpr double kTwoPi = 6.283185307179586;
	for (size_t j = 0; j < n; ++j)
	{
		const double shifted = q[j] - kTwoPi * std::round((q[j] - ref[j]) / kTwoPi);
		const int ji = static_cast<int>(j);
		if (shifted < lowerRad[ji] - 1e-9 || shifted > upperRad[ji] + 1e-9)
		{
			return false;
		}
		q[j] = shifted;
	}
	return true;
}

std::string describeJointNormalizeFailure(const std::vector<double>& q, const std::vector<double>& ref,
										  const QVector<double>& lowerRad, const QVector<double>& upperRad)
{
	constexpr double kTwoPi = 6.283185307179586;
	constexpr double kPi = 3.141592653589793;
	constexpr double kRadToDeg = 180.0 / kPi;
	int worstJ = -1;
	double worstOver = -1.0;
	double qDeg = 0.0;
	double refDeg = 0.0;
	double shiftedDeg = 0.0;
	double loDeg = 0.0;
	double hiDeg = 0.0;
	double foldedAbsDeg = 0.0;
	for (size_t j = 0; j < q.size() && j < ref.size(); ++j)
	{
		const int ji = static_cast<int>(j);
		if (ji >= lowerRad.size() || ji >= upperRad.size())
		{
			continue;
		}
		const double shifted = q[j] - kTwoPi * std::round((q[j] - ref[j]) / kTwoPi);
		const double over = std::max(lowerRad[ji] - shifted, shifted - upperRad[ji]);
		if (over > worstOver)
		{
			worstOver = over;
			worstJ = ji;
			qDeg = q[j] * kRadToDeg;
			refDeg = ref[j] * kRadToDeg;
			shiftedDeg = shifted * kRadToDeg;
			loDeg = lowerRad[ji] * kRadToDeg;
			hiDeg = upperRad[ji] * kRadToDeg;
			double d = q[j] - ref[j];
			d -= kTwoPi * std::round(d / kTwoPi);
			foldedAbsDeg = std::abs(d) * kRadToDeg;
		}
	}
	if (worstJ < 0)
	{
		return QStringLiteral("关节转数折回失败：最接近的整圈超出限位").toStdString();
	}
	const double unfoldAbsDeg = std::abs(qDeg - refDeg);
	if (foldedAbsDeg > 180.0)
	{
		return QStringLiteral("分支跨越被拒：J%1 最短连续转动 %2°（链种子 %3° → IK %4°），超出物理连续性限制。"
							  "折回最近圈得 %5°，限位 [%6°, %7°]。请在相邻指令间插入 PTP 或重新示教。")
			.arg(worstJ + 1)
			.arg(foldedAbsDeg, 0, 'f', 1)
			.arg(refDeg, 0, 'f', 1)
			.arg(qDeg, 0, 'f', 1)
			.arg(shiftedDeg, 0, 'f', 1)
			.arg(loDeg, 0, 'f', 1)
			.arg(hiDeg, 0, 'f', 1)
			.toStdString();
	}
	return QStringLiteral("关节转数折回失败：J%1 链种子 %2°，IK 解 %3°（未折圈差 %4°）。"
						  "折回最近圈得 %5°，超出限位 [%6°, %7°]。连续 LINE 无法跨圈，请改用 PTP 过渡或重新示教。")
		.arg(worstJ + 1)
		.arg(refDeg, 0, 'f', 1)
		.arg(qDeg, 0, 'f', 1)
		.arg(unfoldAbsDeg, 0, 'f', 1)
		.arg(shiftedDeg, 0, 'f', 1)
		.arg(loDeg, 0, 'f', 1)
		.arg(hiDeg, 0, 'f', 1)
		.toStdString();
}

void alignTrajectoryAfterTargetNormalize(RobotInstruction::PlanResult& plan,
										 const std::vector<double>& targetsBeforeNormalize)
{
	if (plan.jointTrajectoryRad.size() < 2U)
	{
		return;
	}
	shiftJointTrajectoryByTargetDelta(plan.jointTrajectoryRad, targetsBeforeNormalize, plan.jointTargetsRad);
	if (!jointTrajectoryTailMatchesTargets(plan.jointTrajectoryRad, plan.jointTargetsRad))
	{
		plan.jointTrajectoryRad.clear();
	}
}

bool applyJointWrapToPlan(RobotInstruction::PlanResult& plan, const std::vector<double>& seedRef,
						  const QVector<double>& lowerRad, const QVector<double>& upperRad)
{
	if (!plan.ok || plan.jointTargetsRad.empty())
	{
		return false;
	}
	if (seedRef.size() != plan.jointTargetsRad.size())
	{
		return true;
	}
	const std::vector<double> before = plan.jointTargetsRad;
	if (!normalizeJointRevolutionsToReference(plan.jointTargetsRad, seedRef, lowerRad, upperRad))
	{
		plan.ok = false;
		plan.summary = describeJointNormalizeFailure(before, seedRef, lowerRad, upperRad);
		return false;
	}
	alignTrajectoryAfterTargetNormalize(plan, before);
	return true;
}
