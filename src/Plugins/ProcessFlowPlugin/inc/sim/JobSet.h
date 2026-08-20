#ifndef PROCESSFLOWPLUGIN_SIM_JOBSET_H
#define PROCESSFLOWPLUGIN_SIM_JOBSET_H

/// @file JobSet.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Job / Operation 工艺定义

#include <QString>
#include <QVector>
#include <vector>

struct OpSpec
{
	int machineNodeId = -1;
	double processTimeSec = 0.0;
	double setupTimeSec = 0.0;
	double priority = 0.0;
	double scrapRate = 0.0;
	double batchSize = 1.0;
	double mtbfSec = 0.0;
	double mttrSec = 0.0;
	double requiredInputs = 1.0;
	QString kind;
	/// 两道 Op 之间路径上的缓冲容量；<0 表示不限
	double interBufferCapacity = -1.0;
};

struct JobTemplate
{
	QString name;
	QVector<OpSpec> ops;
	double dueDateSec = -1.0; // <0 表示按工艺总时长*1.5 自动
	double releaseOffsetSec = 0.0;
};

struct JobSet
{
	QVector<JobTemplate> templates;
};

#endif
