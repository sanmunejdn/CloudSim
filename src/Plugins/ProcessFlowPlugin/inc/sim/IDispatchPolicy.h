#ifndef PROCESSFLOWPLUGIN_SIM_IDISPATCHPOLICY_H
#define PROCESSFLOWPLUGIN_SIM_IDISPATCHPOLICY_H

/// @file IDispatchPolicy.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 机器空闲时的派工策略

#include <QVector>

struct ReadyOpCandidate
{
	int jobId = -1;
	int opSeq = 0;
	double processTimeSec = 0.0;
	double enqueueTime = 0.0;
	double priority = 0.0;
	double dueDateSec = 1.0e12;
	double remainingWorkSec = 0.0;
};

struct DispatchContext
{
	int machineNodeId = -1;
	double now = 0.0;
	QVector<ReadyOpCandidate> candidates;
};

class IDispatchPolicy
{
public:
	virtual ~IDispatchPolicy() = default;
	/// 返回候选下标；<0 表示暂不派工
	virtual int select(const DispatchContext& ctx) const = 0;
};

#endif
