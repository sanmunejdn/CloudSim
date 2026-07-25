#ifndef PROCESSFLOWPLUGIN_SIM_ISTATIONEXECUTOR_H
#define PROCESSFLOWPLUGIN_SIM_ISTATIONEXECUTOR_H

/// @file IStationExecutor.h
/// @brief 工位真实执行预留；数学仿真走 Null 实现

#include <QString>

struct StationBinding
{
	QString backendId;
	QString programId;
};

class IStationExecutor
{
public:
	virtual ~IStationExecutor() = default;
	virtual double beginProcess(int nodeId, int entityId, double cycleTimeSec,
								const StationBinding& binding) = 0;
};

class NullStationExecutor final : public IStationExecutor
{
public:
	double beginProcess(int /*nodeId*/, int /*entityId*/, double cycleTimeSec,
						const StationBinding& /*binding*/) override
	{
		return cycleTimeSec;
	}
};

#endif
