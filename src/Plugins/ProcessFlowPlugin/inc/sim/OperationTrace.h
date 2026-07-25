#ifndef PROCESSFLOWPLUGIN_SIM_OPERATIONTRACE_H
#define PROCESSFLOWPLUGIN_SIM_OPERATIONTRACE_H

/// @file OperationTrace.h
/// @brief 甘特数据源（工序占用区间）

#include <QJsonArray>
#include <QJsonObject>
#include <QVector>

struct OperationTraceItem
{
	int jobId = 0;
	int opSeq = 0;
	int machineNodeId = -1;
	double start = 0.0;
	double end = 0.0;
};

struct OperationTrace
{
	QVector<OperationTraceItem> items;

	QJsonArray toJsonArray() const
	{
		QJsonArray arr;
		for (const OperationTraceItem& it : items)
		{
			QJsonObject o;
			o.insert(QStringLiteral("jobId"), it.jobId);
			o.insert(QStringLiteral("opSeq"), it.opSeq);
			o.insert(QStringLiteral("machineNodeId"), it.machineNodeId);
			o.insert(QStringLiteral("start"), it.start);
			o.insert(QStringLiteral("end"), it.end);
			arr.append(o);
		}
		return arr;
	}
};

#endif
