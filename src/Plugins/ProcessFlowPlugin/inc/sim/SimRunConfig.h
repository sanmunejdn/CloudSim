#ifndef PROCESSFLOWPLUGIN_SIM_SIMRUNCONFIG_H
#define PROCESSFLOWPLUGIN_SIM_SIMRUNCONFIG_H

/// @file SimRunConfig.h
/// @brief DES 运行参数（含班次/到达分布）

#include <QString>

struct ShiftCalendar
{
	bool enabled = false;
	double dayLengthSec = 86400.0;
	double workStartSec = 28800.0; // 08:00
	double workEndSec = 57600.0;   // 16:00
	bool finishInProgressOvertime = true;
};

struct SimRunConfig
{
	double horizonSec = 3600.0;
	double warmupSec = 0.0;
	QString policy = QStringLiteral("fifo");
	double defaultInterarrivalSec = 30.0;
	/// fixed | exponential
	QString arrivalMode = QStringLiteral("fixed");
	int maxJobs = 10000;
	unsigned int seed = 1;
	bool untilAllJobsDone = false;
	ShiftCalendar shift;
	/// simOnly | drivePreview
	QString executorMode = QStringLiteral("simOnly");
	bool openGanttAfterRun = false;
	bool includeCompareTraces = false;
};

#endif
