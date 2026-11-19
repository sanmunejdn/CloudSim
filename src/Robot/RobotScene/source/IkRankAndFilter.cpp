/// @file IkRankAndFilter.cpp
/// @brief 轴配置硬门 / 腕连续 / Rank

#include "IkRankAndFilter.h"

#include "IkSolveMetrics.h"

#include <algorithm>
#include <cmath>

namespace RobotInstruction
{
int elbowSignFromQ(const std::vector<double>& q, const int elbowIdx)
{
	if (elbowIdx < 0 || elbowIdx >= static_cast<int>(q.size()))
	{
		return 0;
	}
	return q[static_cast<size_t>(elbowIdx)] >= 0.0 ? 0 : 1;
}

int softCfgMismatchScore(const std::vector<double>& q, const std::vector<std::string>& jointNames,
						 const std::vector<double>& classRef, const MotionAxisConfiguration& cfg, const int elbowOv,
						 const int wristOv)
{
	if (!motionAxisConfigurationRequiresConstraint(cfg))
	{
		return 0;
	}
	JointConfigurationClass want{};
	cfg.resolveEffective(want);
	const JointConfigurationClass observed = classifyJointConfiguration(q, jointNames, &classRef, elbowOv, wristOv);
	int score = 0;
	if (want.elbow != ElbowPosture::Auto && want.elbow != observed.elbow)
	{
		++score;
	}
	if (want.wrist != WristPosture::Auto && want.wrist != observed.wrist)
	{
		++score;
	}
	if (want.arm != ArmPosture::Auto && want.arm != observed.arm)
	{
		++score;
	}
	return score;
}

double softLimitProximityPenalty(const std::vector<double>& q, const QVector<double>& lower,
								 const QVector<double>& upper)
{
	const int n = std::min(static_cast<int>(q.size()), std::min(lower.size(), upper.size()));
	double pen = 0.0;
	for (int i = 0; i < n; ++i)
	{
		const double span = upper[i] - lower[i];
		if (!(span > 1e-6) || span > 1e6)
		{
			continue;
		}
		const double margin = std::min(q[static_cast<size_t>(i)] - lower[i], upper[i] - q[static_cast<size_t>(i)]);
		const double zone = 0.1 * span;
		if (margin < zone)
		{
			pen += zone - margin;
		}
	}
	return pen;
}

bool isIkCandidateBetter(const IkCandidate& c, const IkCandidate& best, const IkRankMode rankMode)
{
	if (c.collisionCost + 1e-9 < best.collisionCost)
	{
		return true;
	}
	if (c.collisionCost > best.collisionCost + 1e-9)
	{
		return false;
	}
	if (c.softCfg < best.softCfg)
	{
		return true;
	}
	if (c.softCfg > best.softCfg)
	{
		return false;
	}
	const bool cCont = c.wristJump <= kWristContinuityGateRad;
	const bool bCont = best.wristJump <= kWristContinuityGateRad;
	if (cCont != bCont)
	{
		return cCont;
	}
	if (rankMode == IkRankMode::Speed)
	{
		return c.seedOrder < best.seedOrder;
	}
	if (rankMode == IkRankMode::Manip)
	{
		if (c.manip > best.manip + 1e-12)
		{
			return true;
		}
		if (c.manip < best.manip - 1e-12)
		{
			return false;
		}
	}
	if (c.wristJump < best.wristJump - 1e-9)
	{
		return true;
	}
	if (c.wristJump > best.wristJump + 1e-9)
	{
		return false;
	}
	const double cCost = c.dist + c.softPen;
	const double bCost = best.dist + best.softPen;
	if (cCost < bCost - 1e-9)
	{
		return true;
	}
	if (cCost > bCost + 1e-9)
	{
		return false;
	}
	if (c.manip > best.manip + 1e-12)
	{
		return true;
	}
	if (c.manip < best.manip - 1e-12)
	{
		return false;
	}
	const size_t n = std::min(c.q.size(), best.q.size());
	for (size_t i = 0; i < n; ++i)
	{
		if (c.q[i] < best.q[i] - 1e-12)
		{
			return true;
		}
		if (c.q[i] > best.q[i] + 1e-12)
		{
			return false;
		}
	}
	return c.q.size() < best.q.size();
}

IkSolveResult rankAndSelectIkCandidates(std::vector<IkCandidate>& converged, const IkSelectionPolicy& policy,
										const MotionAxisConfiguration& cfg, const std::vector<std::string>& jointNames,
										const std::vector<double>& classRef, const bool sphericalWrist,
										const int elbowOverride, const int wristOverride, const bool budgetExhausted,
										const std::string& lastSeedFail,
										const std::function<double(const std::vector<double>&)>& manipFn)
{
	IkSolveResult result;
	std::vector<IkCandidate> matching;
	const bool hardClass = policy.requireMatchesClass && sphericalWrist;
	if (hardClass)
	{
		for (const IkCandidate& c : converged)
		{
			const JointConfigurationClass observed =
				classifyJointConfiguration(c.q, jointNames, &classRef, elbowOverride, wristOverride);
			if (cfg.matchesClass(observed))
			{
				matching.push_back(c);
			}
		}
	}
	std::vector<IkCandidate>& poolBase = hardClass ? matching : converged;
	if (poolBase.empty())
	{
		if (budgetExhausted && converged.empty())
		{
			result.failKind = IkFailKind::Budget;
			result.message = "IK预算耗尽";
		}
		else if (hardClass && !converged.empty())
		{
			result.failKind = IkFailKind::AxisClass;
			result.message = "无满足轴配置的IK解（已收敛但支/转数不匹配）";
		}
		else if (!lastSeedFail.empty())
		{
			result.message = lastSeedFail;
			result.failKind = classifyIkFailMessage(lastSeedFail);
		}
		else
		{
			result.failKind = IkFailKind::Residual;
			result.message = "IK未收敛";
		}
		return result;
	}

	std::vector<IkCandidate> continuousOnly;
	std::vector<IkCandidate>* pool = &poolBase;
	if (policy.requireWristContinuity)
	{
		for (const IkCandidate& c : poolBase)
		{
			if (c.wristJump <= kWristContinuityGateRad)
			{
				continuousOnly.push_back(c);
			}
		}
		if (continuousOnly.empty())
		{
			result.failKind = IkFailKind::Wrist;
			result.message = "轴配置解腕部跳变过大（相对种子超过连续性门限）";
			return result;
		}
		pool = &continuousOnly;
	}

	if (policy.rankMode == IkRankMode::Distance && policy.manipOnDistanceTie && pool->size() >= 2 && manipFn)
	{
		double bestDist = 1e30;
		for (const IkCandidate& c : *pool)
		{
			bestDist = std::min(bestDist, c.dist);
		}
		for (IkCandidate& c : *pool)
		{
			if (c.dist <= bestDist + 1e-3 && c.manip <= 0.0)
			{
				c.manip = manipFn(c.q);
			}
		}
	}

	const IkCandidate* best = &pool->front();
	for (const IkCandidate& c : *pool)
	{
		if (isIkCandidateBetter(c, *best, policy.rankMode))
		{
			best = &c;
		}
	}
	result.q = best->q;
	result.branchId = best->branchId;
	result.failKind = IkFailKind::None;
	return result;
}

} // namespace RobotInstruction
