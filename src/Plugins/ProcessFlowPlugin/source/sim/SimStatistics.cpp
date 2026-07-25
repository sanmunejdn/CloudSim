/// @file SimStatistics.cpp
/// @brief 统计序列化

#include "sim/SimStatistics.h"

#include <QJsonArray>
#include <QTextStream>

QJsonObject SimStatistics::toJson() const
{
	QJsonObject root;
	root.insert(QStringLiteral("horizonSec"), horizonSec);
	root.insert(QStringLiteral("warmupSec"), warmupSec);
	root.insert(QStringLiteral("makespan"), makespan);
	root.insert(QStringLiteral("completedJobs"), completedJobs);
	root.insert(QStringLiteral("scrappedJobs"), scrappedJobs);
	root.insert(QStringLiteral("releasedJobs"), releasedJobs);
	root.insert(QStringLiteral("throughputPerHour"), throughputPerHour);
	root.insert(QStringLiteral("avgWip"), avgWip);
	root.insert(QStringLiteral("maxWip"), maxWip);
	root.insert(QStringLiteral("bottleneckNodeId"), bottleneckNodeId);
	root.insert(QStringLiteral("bottleneckTitle"), bottleneckTitle);

	QJsonArray machinesArr;
	for (const MachineStat& m : machines)
	{
		QJsonObject o;
		o.insert(QStringLiteral("nodeId"), m.nodeId);
		o.insert(QStringLiteral("title"), m.title);
		o.insert(QStringLiteral("utilization"), m.utilization);
		o.insert(QStringLiteral("busyTimeSec"), m.busyTimeSec);
		o.insert(QStringLiteral("blockedTimeSec"), m.blockedTimeSec);
		o.insert(QStringLiteral("avgQueueLen"), m.avgQueueLen);
		o.insert(QStringLiteral("maxQueueLen"), m.maxQueueLen);
		machinesArr.append(o);
	}
	root.insert(QStringLiteral("machines"), machinesArr);

	QJsonArray buffersArr;
	for (const BufferStat& b : buffers)
	{
		QJsonObject o;
		o.insert(QStringLiteral("nodeId"), b.nodeId);
		o.insert(QStringLiteral("title"), b.title);
		o.insert(QStringLiteral("avgInventory"), b.avgInventory);
		o.insert(QStringLiteral("maxInventory"), b.maxInventory);
		o.insert(QStringLiteral("fullCount"), b.fullCount);
		buffersArr.append(o);
	}
	root.insert(QStringLiteral("buffers"), buffersArr);
	root.insert(QStringLiteral("operationTrace"), trace.toJsonArray());
	return root;
}

QString SimStatistics::toCsv() const
{
	QString out;
	QTextStream ts(&out);
	ts << "section,key,value\n";
	ts << "global,horizonSec," << horizonSec << "\n";
	ts << "global,makespan," << makespan << "\n";
	ts << "global,completedJobs," << completedJobs << "\n";
	ts << "global,scrappedJobs," << scrappedJobs << "\n";
	ts << "global,throughputPerHour," << throughputPerHour << "\n";
	ts << "global,avgWip," << avgWip << "\n";
	ts << "global,maxWip," << maxWip << "\n";
	ts << "global,bottleneckNodeId," << bottleneckNodeId << "\n";
	ts << "global,bottleneckTitle," << bottleneckTitle << "\n";
	for (const MachineStat& m : machines)
	{
		ts << "machine," << m.nodeId << "_util," << m.utilization << "\n";
		ts << "machine," << m.nodeId << "_busy," << m.busyTimeSec << "\n";
		ts << "machine," << m.nodeId << "_blocked," << m.blockedTimeSec << "\n";
	}
	ts << "jobId,opSeq,machineNodeId,start,end\n";
	for (const OperationTraceItem& it : trace.items)
	{
		ts << it.jobId << "," << it.opSeq << "," << it.machineNodeId << "," << it.start << "," << it.end << "\n";
	}
	return out;
}
