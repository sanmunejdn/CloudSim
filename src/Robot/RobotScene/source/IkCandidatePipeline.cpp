/// @file IkCandidatePipeline.cpp
/// @brief Seed→Solver→Filter→Rank→Budget（行为锁自 Controller）

#include "IkCandidatePipeline.h"

#include "IkRankAndFilter.h"
#include "IkSeedExpand.h"
#include "IkSeedProvider.h"
#include "IkSolveMetrics.h"
#include "PoseIkSolverRegistry.h"

#include <QElapsedTimer>
#include <QString>

#include <algorithm>
#include <atomic>
#include <future>
#include <mutex>

namespace RobotInstruction
{
IkSolveResult runIkCandidatePipeline(const IkPipelineContext& ctx)
{
	QElapsedTimer wall;
	wall.start();
	IkSolveResult result;
	if (ctx.qSeed.empty())
	{
		result.failKind = IkFailKind::NoSeed;
		result.message = "缺少IK种子关节角";
		recordIkSolveFailure(result.failKind, static_cast<int>(wall.elapsed()));
		return result;
	}
	if (!ctx.armSolve)
	{
		result.failKind = IkFailKind::Other;
		result.message = "缺少臂IK求解回调";
		recordIkSolveFailure(result.failKind, static_cast<int>(wall.elapsed()));
		return result;
	}

	IkSelectionPolicy effectivePolicy = ctx.policy;
	std::vector<IkCandidate> converged;
	std::string lastSeedFail;
	bool budgetExhausted = false;

	auto runSearchPass = [&](IkSelectionPolicy& pol)
	{
		converged.clear();
		lastSeedFail.clear();
		budgetExhausted = false;
		std::atomic<int> seedOrder{0};
		std::mutex poolMu;

		std::vector<std::vector<double>> seeds;
		if (pol.mode == IkSolveMode::Search)
		{
			seeds = buildIkSeedVariants(ctx.qSeed, ctx.jointNames, &ctx.cfg, ctx.sphericalWrist, ctx.elbowOverride,
										ctx.wristOverride);
			std::sort(seeds.begin(), seeds.end(),
					  [&](const std::vector<double>& a, const std::vector<double>& b)
					  {
						  return UrdfRobotLoader::wrappedJointDistance(a, ctx.qSeed) <
								 UrdfRobotLoader::wrappedJointDistance(b, ctx.qSeed);
					  });
		}
		else
		{
			seeds.push_back(ctx.qSeed);
		}
		if (pol.maxSeeds > 0 && static_cast<int>(seeds.size()) > pol.maxSeeds)
		{
			seeds.resize(static_cast<size_t>(pol.maxSeeds));
		}

		QElapsedTimer budget;
		budget.start();
		const bool useBudget = pol.budgetMs > 0;
		const int elbowIdx = ctx.elbowOverride >= 0 ? ctx.elbowOverride : 2;

		auto pushCandidate = [&](std::vector<double> qTry, const int branchId, const int order,
								 const char* solverName)
		{
			IkCandidate c;
			c.dist = UrdfRobotLoader::wrappedJointDistance(qTry, ctx.qSeed);
			c.wristJump = wristConfigurationJumpRad(qTry, ctx.qSeed, ctx.jointNames);
			c.collisionCost = ctx.collisionFn ? ctx.collisionFn(qTry) : 0.0;
			if (c.collisionCost >= 1e6)
			{
				return;
			}
			c.manip = pol.rankMode == IkRankMode::Manip && ctx.manipFn ? ctx.manipFn(qTry) : 0.0;
			c.softCfg =
				softCfgMismatchScore(qTry, ctx.jointNames, ctx.classRef, ctx.cfg, ctx.elbowOverride, ctx.wristOverride);
			if (pol.softLimitSingularityPenalty)
			{
				c.softPen = softLimitProximityPenalty(qTry, ctx.limitLower, ctx.limitUpper);
			}
			c.seedOrder = order;
			c.branchId = branchId;
			if (solverName)
			{
				c.solverName = solverName;
			}
			c.q = std::move(qTry);
			std::lock_guard<std::mutex> lock(poolMu);
			converged.push_back(std::move(c));
		};

		auto tryOneSeed = [&](const std::vector<double>& seed, const bool numericalOnly, std::string* seedFailOut)
		{
			if (useBudget && budget.elapsed() >= pol.budgetMs)
			{
				budgetExhausted = true;
				return;
			}
			const int order = seedOrder.fetch_add(1) + 1;
			std::string seedFail;
			const bool raceBoth = pol.raceAnalyticalAndNumerical && !numericalOnly;
			std::vector<double> qTry =
				ctx.armSolve(seed, numericalOnly, raceBoth, pol.seedAttractionWeight, &seedFail);
			if (qTry.empty())
			{
				if (seedFailOut && !seedFail.empty())
				{
					*seedFailOut = std::move(seedFail);
				}
				return;
			}
			pushCandidate(std::move(qTry), -1, order, numericalOnly ? "dls" : "arm");
		};

		auto trySeedPass = [&](const bool numericalOnly)
		{
			// 球形腕：Registry 收解析多腕支；外轴激活时跳过（目标需先换算）
			if (!numericalOnly && ctx.sphericalWrist && !ctx.externalContextActive &&
				ctx.machineProfile.analyticalFamily != UrdfRobotLoader::AnalyticalIkFamily::None &&
				ctx.analyticalInput.valid)
			{
				UrdfRobotLoader::UrdfIkSolverOptions anaOpt{};
				anaOpt.maxIterations = 80;
				std::string anaFail;
				const auto anaCands = UrdfRobotLoader::collectPoseIkCandidates(
					QString::fromStdString(ctx.analyticalInput.urdfPath),
					QString::fromStdString(ctx.analyticalInput.ikLinkName), ctx.analyticalInput.poseTarget, ctx.qSeed,
					anaOpt, true, false, &anaFail);
				for (const auto& ac : anaCands)
				{
					if (useBudget && budget.elapsed() >= pol.budgetMs)
					{
						budgetExhausted = true;
						break;
					}
					const int order = seedOrder.fetch_add(1) + 1;
					const int wristBranch = ac.branchId >= 0 ? ac.branchId : 0;
					const int bid = encodeIkBranchId(elbowSignFromQ(ac.q, elbowIdx), wristBranch);
					pushCandidate(ac.q, bid, order, ac.solverName ? ac.solverName : "analytical");
				}
				if (!anaCands.empty() && pol.rankMode == IkRankMode::Speed && !pol.requireWristContinuity &&
					!pol.requireMatchesClass)
				{
					return;
				}
			}

			const bool doParallel = pol.allowParallelSeeds && !numericalOnly && !ctx.externalContextActive &&
									pol.mode == IkSolveMode::Search && seeds.size() > 1;
			if (doParallel)
			{
				constexpr int kMaxWorkers = 4;
				for (size_t i = 0; i < seeds.size();)
				{
					if (useBudget && budget.elapsed() >= pol.budgetMs)
					{
						budgetExhausted = true;
						break;
					}
					std::vector<std::future<std::pair<std::vector<double>, std::string>>> futs;
					futs.reserve(static_cast<size_t>(kMaxWorkers));
					for (int w = 0; w < kMaxWorkers && i < seeds.size(); ++w, ++i)
					{
						const std::vector<double> seedCopy = seeds[i];
						const bool raceBoth = pol.raceAnalyticalAndNumerical;
						const double attract = pol.seedAttractionWeight;
						const IkArmPoseSolveFn armSolve = ctx.armSolve;
						futs.push_back(std::async(std::launch::async,
												  [armSolve, seedCopy, raceBoth, attract]()
												  {
													  std::string fr;
													  std::vector<double> q =
														  armSolve(seedCopy, false, raceBoth, attract, &fr);
													  return std::make_pair(std::move(q), std::move(fr));
												  }));
					}
					for (auto& fut : futs)
					{
						auto pair = fut.get();
						if (pair.first.empty())
						{
							if (!pair.second.empty())
							{
								lastSeedFail = std::move(pair.second);
							}
							continue;
						}
						const int order = seedOrder.fetch_add(1) + 1;
						pushCandidate(std::move(pair.first), -1, order, "arm");
					}
				}
				return;
			}

			for (const std::vector<double>& seed : seeds)
			{
				std::string seedFail;
				tryOneSeed(seed, numericalOnly, &seedFail);
				if (!seedFail.empty())
				{
					lastSeedFail = std::move(seedFail);
				}
				if (budgetExhausted)
				{
					break;
				}
				if (pol.rankMode == IkRankMode::Speed && !pol.requireWristContinuity && !pol.requireMatchesClass &&
					!converged.empty())
				{
					break;
				}
			}
		};

		trySeedPass(false);
		if (converged.empty() && pol.raceNumericalFallback && !(useBudget && budget.elapsed() >= pol.budgetMs))
		{
			trySeedPass(true);
		}
		else if (converged.empty() && useBudget && budget.elapsed() >= pol.budgetMs)
		{
			budgetExhausted = true;
		}
	};

	runSearchPass(effectivePolicy);
	std::string escalateTag;
	if (converged.empty() && ctx.dragProfile)
	{
		effectivePolicy.mode = IkSolveMode::Search;
		effectivePolicy.budgetMs = 120;
		effectivePolicy.maxSeeds = 16;
		runSearchPass(effectivePolicy);
		if (!converged.empty())
		{
			escalateTag = " [ikEscalate=preview]";
		}
	}

	// Phase E：空池才 Budget；有收敛候选则进入 Rank（与原路径一致）
	if (budgetExhausted && !converged.empty())
	{
		budgetExhausted = false;
	}
	std::vector<double> partialQ;
	if (!converged.empty())
	{
		partialQ = converged.front().q;
	}

	result = rankAndSelectIkCandidates(converged, effectivePolicy, ctx.cfg, ctx.jointNames, ctx.classRef,
									   ctx.sphericalWrist, ctx.elbowOverride, ctx.wristOverride, budgetExhausted,
									   lastSeedFail, ctx.manipFn);
	result.escalateTag = std::move(escalateTag);
	result.elapsedMs = static_cast<int>(wall.elapsed());
	result.candidatesTried = static_cast<int>(converged.size());
	result.bestPartialQ = result.ok() ? result.q : std::move(partialQ);
	if (result.ok())
	{
		recordIkSolveSuccess(result.elapsedMs);
	}
	else
	{
		recordIkSolveFailure(result.failKind, result.elapsedMs);
	}
	return result;
}

} // namespace RobotInstruction
