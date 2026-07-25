#ifndef PROCESSFLOWPLUGIN_SIM_DESENGINE_H
#define PROCESSFLOWPLUGIN_SIM_DESENGINE_H

/// @file DesEngine.h
/// @brief 离散事件仿真引擎

#include "IDispatchPolicy.h"
#include "IStationExecutor.h"
#include "JobSet.h"
#include "PlantGraph.h"
#include "SimRunConfig.h"
#include "SimStatistics.h"

#include <atomic>
#include <memory>

class DesEngine
{
public:
	DesEngine();

	void setDispatchPolicy(std::unique_ptr<IDispatchPolicy> policy);
	void setStationExecutor(std::unique_ptr<IStationExecutor> executor);

	SimStatistics run(const PlantGraph& plant, const JobSet& jobSet, double interarrivalSec,
					  const SimRunConfig& config, std::atomic_bool* cancelFlag = nullptr);

private:
	std::unique_ptr<IDispatchPolicy> m_policy;
	std::unique_ptr<IStationExecutor> m_executor;
};

#endif
