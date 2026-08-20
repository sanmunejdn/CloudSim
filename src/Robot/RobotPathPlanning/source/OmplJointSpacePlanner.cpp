/// @file OmplJointSpacePlanner.cpp
/// @brief OMPL BIT* / InformedRRT* / RRT* / RRTConnect（CLOUDSIM_HAS_OMPL）

#include "OmplJointSpacePlanner.h"

#if defined(CLOUDSIM_HAS_OMPL)

#include <ompl/base/ScopedState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/geometric/planners/informedtrees/BITstar.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/util/RandomNumbers.h>

#include <cmath>
#include <string>

namespace robot_path
{
namespace detail
{
namespace
{

namespace ob = ompl::base;
namespace og = ompl::geometric;

class CloudSimStateValidityChecker : public ob::StateValidityChecker
{
public:
	CloudSimStateValidityChecker(const ob::SpaceInformationPtr& si, const PlanRequest& req, const JointLimits& lim)
		: ob::StateValidityChecker(si), m_req(req), m_lim(lim)
	{
	}

	bool isValid(const ob::State* state) const override
	{
		const auto* rv = state->as<ob::RealVectorStateSpace::StateType>();
		const unsigned int dim = si_->getStateSpace()->getDimension();
		std::vector<double> q(dim);
		for (unsigned int i = 0; i < dim; ++i)
			q[i] = rv->values[i];
		return isStateValid(m_req, m_lim, q);
	}

private:
	PlanRequest m_req;
	JointLimits m_lim;
};

class CloudSimMotionValidator : public ob::MotionValidator
{
public:
	CloudSimMotionValidator(const ob::SpaceInformationPtr& si, const PlanRequest& req, const JointLimits& lim)
		: ob::MotionValidator(si), m_req(req), m_lim(lim)
	{
	}

	bool checkMotion(const ob::State* s1, const ob::State* s2) const override
	{
		return checkSegment(s1, s2);
	}

	bool checkMotion(const ob::State* s1, const ob::State* s2, std::pair<ob::State*, double>& lastValid) const override
	{
		if (checkSegment(s1, s2))
			return true;
		lastValid.first = nullptr;
		lastValid.second = 0.0;
		return false;
	}

private:
	bool checkSegment(const ob::State* s1, const ob::State* s2) const
	{
		const auto* a = s1->as<ob::RealVectorStateSpace::StateType>();
		const auto* b = s2->as<ob::RealVectorStateSpace::StateType>();
		const unsigned int dim = si_->getStateSpace()->getDimension();
		std::vector<double> qa(dim), qb(dim);
		for (unsigned int i = 0; i < dim; ++i)
		{
			qa[i] = a->values[i];
			qb[i] = b->values[i];
		}
		return isSegmentValid(m_req, m_lim, qa, qb, m_req.options.longestValidSegmentRad);
	}

	PlanRequest m_req;
	JointLimits m_lim;
};

std::string normalizePlannerId(const std::string& id)
{
	if (id == "BITstar" || id == "BIT*")
		return "BITstar";
	if (id == "RRTConnect" || id == "RRTstar" || id == "RRT*" || id == "InformedRRTstar" || id == "InformedRRT*")
		return id == "RRT*" ? "RRTstar" : (id == "InformedRRT*" ? "InformedRRTstar" : id);
	return "BITstar";
}

double pathLengthL2(const std::vector<std::vector<double>>& path)
{
	double len = 0.0;
	for (std::size_t i = 1; i < path.size(); ++i)
	{
		double s = 0.0;
		for (std::size_t j = 0; j < path[i].size() && j < path[i - 1].size(); ++j)
		{
			const double d = path[i][j] - path[i - 1][j];
			s += d * d;
		}
		len += std::sqrt(s);
	}
	return len;
}

} // namespace

bool planJointSpaceOmpl(const PlanRequest& req, const JointLimits& lim, const std::vector<double>& goalQ, PathResult& out)
{
	const std::size_t dim = lim.lowerRad.size();
	if (req.startJointRad.size() != dim || goalQ.size() != dim)
	{
		out.errMsg = "joint dimension mismatch";
		return false;
	}

	auto space = std::make_shared<ob::RealVectorStateSpace>(dim);
	ob::RealVectorBounds bounds(static_cast<unsigned int>(dim));
	for (std::size_t i = 0; i < dim; ++i)
	{
		bounds.setLow(static_cast<unsigned int>(i), lim.lowerRad[i]);
		bounds.setHigh(static_cast<unsigned int>(i), lim.upperRad[i]);
	}
	space->setBounds(bounds);

	auto si = std::make_shared<ob::SpaceInformation>(space);
	si->setStateValidityChecker(std::make_shared<CloudSimStateValidityChecker>(si, req, lim));
	si->setMotionValidator(std::make_shared<CloudSimMotionValidator>(si, req, lim));
	si->setup();

	ob::ScopedState<> start(space);
	ob::ScopedState<> goal(space);
	for (std::size_t i = 0; i < dim; ++i)
	{
		start[static_cast<unsigned int>(i)] = req.startJointRad[i];
		goal[static_cast<unsigned int>(i)] = goalQ[i];
	}

	ompl::RNG::setSeed(req.options.rngSeed);

	auto pdef = std::make_shared<ob::ProblemDefinition>(si);
	pdef->setStartAndGoalStates(start, goal);

	// 参考 ompl/ompl demos/OptimalPlanning.cpp：路径长度 + cost-to-go + 阈值 0（用满时限）
	// https://github.com/ompl/ompl/blob/main/demos/OptimalPlanning.cpp
	auto optObj = std::make_shared<ob::PathLengthOptimizationObjective>(si);
	optObj->setCostThreshold(ob::Cost(0.0));
	optObj->setCostToGoHeuristic(&ob::goalRegionCostToGo);
	pdef->setOptimizationObjective(optObj);

	const std::string plannerId = normalizePlannerId(req.options.plannerId);
	ob::PlannerPtr planner;
	if (plannerId == "RRTConnect")
	{
		planner = std::make_shared<og::RRTConnect>(si);
	}
	else if (plannerId == "RRTstar")
	{
		auto* rrtStar = new og::RRTstar(si);
		rrtStar->setDelayCC(true);
		rrtStar->setRewireFactor(1.1);
		planner.reset(rrtStar);
	}
	else if (plannerId == "InformedRRTstar")
	{
		// 构造函数已打开 informed sampling；勿再 setFocusSearch（会打开 new-state rejection）
		auto* informed = new og::InformedRRTstar(si);
		informed->setDelayCC(true);
		informed->setRewireFactor(1.1);
		planner.reset(informed);
	}
	else
	{
		// BIT*：批量启发树，同预算下通常比 Informed RRT* 更逼近最优长度
		// https://github.com/ompl/ompl （BITstar）/ IJRR 2020 Gammell et al.
		auto* bit = new og::BITstar(si);
		bit->setUseKNearest(true);
		bit->setRewireFactor(1.1);
		bit->setSamplesPerBatch(100);
		planner.reset(bit);
	}
	planner->setProblemDefinition(pdef);
	planner->setup();

	const ob::PlannerTerminationCondition ptc(ob::timedPlannerTerminationCondition(req.options.planningTimeSec));
	const ob::PlannerStatus st = planner->solve(ptc);
	if (!st)
	{
		out.errMsg = std::string("OMPL planner failed: ") + st.asString();
		return false;
	}

	og::PathGeometric path(si);
	const ob::PathPtr sol = pdef->getSolutionPath();
	if (!sol)
	{
		out.errMsg = "OMPL returned no solution path";
		return false;
	}
	path = *sol->as<og::PathGeometric>();
	const double lenBefore = path.length();

	// 稠密化后再 simplifyMax，便于捷径跨段删点（MoveIt / OMPL PathSimplifier 惯例）
	path.interpolate();
	og::PathSimplifier simplifier(si, pdef->getGoal(), optObj);
	(void)simplifier.simplifyMax(path);
	const double lenAfter = path.length();

	if (path.getStateCount() < 2)
	{
		out.errMsg = "OMPL path too short";
		return false;
	}

	out.jointTrajectoryRad.clear();
	out.jointTrajectoryRad.reserve(path.getStateCount() + 1);
	for (std::size_t i = 0; i < path.getStateCount(); ++i)
	{
		const auto* rv = path.getState(i)->as<ob::RealVectorStateSpace::StateType>();
		std::vector<double> q(dim);
		for (std::size_t j = 0; j < dim; ++j)
			q[j] = rv->values[j];
		out.jointTrajectoryRad.push_back(std::move(q));
	}

	auto distL2 = [](const std::vector<double>& a, const std::vector<double>& b) {
		double s = 0.0;
		for (std::size_t i = 0; i < a.size(); ++i)
		{
			const double d = a[i] - b[i];
			s += d * d;
		}
		return std::sqrt(s);
	};
	const double endDq = distL2(out.jointTrajectoryRad.back(), goalQ);
	constexpr double kGoalSnapRad = 1e-3;
	if (endDq > kGoalSnapRad)
	{
		if (!isSegmentValid(req, lim, out.jointTrajectoryRad.back(), goalQ, req.options.longestValidSegmentRad))
		{
			out.errMsg = std::string("OMPL path does not reach goal (") + st.asString() + ", dq=" +
						 std::to_string(endDq) + ")";
			out.ok = false;
			out.jointTrajectoryRad.clear();
			return false;
		}
		out.jointTrajectoryRad.push_back(goalQ);
	}
	else
	{
		out.jointTrajectoryRad.back() = goalQ;
	}

	out.ok = true;
	out.plannerName = plannerId;
	return true;
}

} // namespace detail
} // namespace robot_path

#endif // CLOUDSIM_HAS_OMPL
