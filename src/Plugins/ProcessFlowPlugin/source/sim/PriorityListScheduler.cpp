/// @file PriorityListScheduler.cpp
/// @brief 轻量离线启发式，输出推荐 DES 策略

#include "sim/IScheduler.h"

Schedule PriorityListScheduler::solve(const JobSet& jobs, const PlantGraph& /*plant*/, const SolveConfig& cfg)
{
	Schedule out;
	if (jobs.templates.isEmpty())
	{
		out.message = QStringLiteral("JobSet empty");
		return out;
	}
	double total = 0.0;
	int ops = 0;
	for (const JobTemplate& t : jobs.templates)
	{
		for (const OpSpec& op : t.ops)
		{
			total += op.setupTimeSec + op.processTimeSec * std::max(1.0, op.batchSize);
			++ops;
		}
	}
	QString policy = cfg.objective.trimmed().toLower();
	if (policy != QStringLiteral("spt") && policy != QStringLiteral("lpt") && policy != QStringLiteral("edd") &&
		policy != QStringLiteral("cr") && policy != QStringLiteral("fifo"))
	{
		policy = (ops > 0 && total / ops > 60.0) ? QStringLiteral("lpt") : QStringLiteral("spt");
	}
	out.ok = true;
	out.recommendedPolicy = policy;
	out.message = QStringLiteral("recommend policy=%1 (ops=%2 totalWork=%3)")
					  .arg(policy)
					  .arg(ops)
					  .arg(total, 0, 'f', 1);
	return out;
}
