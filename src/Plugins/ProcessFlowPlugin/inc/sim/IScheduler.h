#ifndef PROCESSFLOWPLUGIN_SIM_ISCHEDULER_H
#define PROCESSFLOWPLUGIN_SIM_ISCHEDULER_H

/// @file IScheduler.h
/// @brief 静态排程：PriorityList 启发式；CP-SAT 可后续替换

#include "JobSet.h"
#include "PlantGraph.h"

#include <QString>
#include <QVector>

struct SolveConfig
{
	double timeLimitSec = 30.0;
	/// spt | edd | lpt — 决定推荐仿真策略
	QString objective = QStringLiteral("spt");
};

struct Schedule
{
	bool ok = false;
	QString recommendedPolicy;
	QString message;
};

class IScheduler
{
public:
	virtual ~IScheduler() = default;
	virtual Schedule solve(const JobSet& jobs, const PlantGraph& plant, const SolveConfig& cfg) = 0;
};

/// 无求解器时：按工序总工时选推荐派工策略，供「优化后仿真」
class PriorityListScheduler final : public IScheduler
{
public:
	Schedule solve(const JobSet& jobs, const PlantGraph& plant, const SolveConfig& cfg) override;
};

#endif
