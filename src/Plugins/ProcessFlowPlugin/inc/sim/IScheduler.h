#ifndef PROCESSFLOWPLUGIN_SIM_ISCHEDULER_H
#define PROCESSFLOWPLUGIN_SIM_ISCHEDULER_H

/// @file IScheduler.h
/// @brief 静态日程求解预留（CP/RL 后期实现）

#include "JobSet.h"
#include "PlantGraph.h"

struct SolveConfig
{
	double timeLimitSec = 30.0;
};

struct Schedule
{
	bool ok = false;
};

class IScheduler
{
public:
	virtual ~IScheduler() = default;
	virtual Schedule solve(const JobSet& jobs, const PlantGraph& plant, const SolveConfig& cfg) = 0;
};

#endif
