#ifndef PROCESSFLOWPLUGIN_SIM_SIMSTATISTICS_H
#define PROCESSFLOWPLUGIN_SIM_SIMSTATISTICS_H

/// @file SimStatistics.h
/// @brief 仿真统计与导出

#include "OperationTrace.h"

#include <QJsonObject>
#include <QString>
#include <QVector>

struct MachineStat
{
	int nodeId = -1;
	QString title;
	double utilization = 0.0;
	double busyTimeSec = 0.0;
	double blockedTimeSec = 0.0;
	double avgQueueLen = 0.0;
	int maxQueueLen = 0;
};

struct BufferStat
{
	int nodeId = -1;
	QString title;
	double avgInventory = 0.0;
	double maxInventory = 0.0;
	int fullCount = 0;
};

struct PolicyCompareRow
{
	QString policy;
	double makespan = 0.0;
	int completed = 0;
	double throughput = 0.0;
	QString bottleneck;
};

struct SimStatistics
{
	double horizonSec = 0.0;
	double warmupSec = 0.0;
	double makespan = 0.0;
	int completedJobs = 0;
	int scrappedJobs = 0;
	int releasedJobs = 0;
	double throughputPerHour = 0.0;
	double avgWip = 0.0;
	double maxWip = 0.0;
	int bottleneckNodeId = -1;
	QString bottleneckTitle;
	QVector<MachineStat> machines;
	QVector<BufferStat> buffers;
	OperationTrace trace;

	QJsonObject toJson() const;
	QString toCsv() const;
};

#endif
