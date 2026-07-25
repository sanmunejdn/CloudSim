#ifndef PROCESSFLOWPLUGIN_SIM_SIMRUNCONFIG_H
#define PROCESSFLOWPLUGIN_SIM_SIMRUNCONFIG_H

/// @file SimRunConfig.h
/// @brief DES 运行参数

#include <QString>

struct SimRunConfig
{
	double horizonSec = 3600.0;
	double warmupSec = 0.0;
	QString policy = QStringLiteral("fifo"); // fifo | spt
	double defaultInterarrivalSec = 30.0;
	int maxJobs = 10000;
	unsigned int seed = 1;
	bool untilAllJobsDone = false;
};

#endif
