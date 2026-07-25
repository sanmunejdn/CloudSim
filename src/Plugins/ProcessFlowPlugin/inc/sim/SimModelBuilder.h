#ifndef PROCESSFLOWPLUGIN_SIM_SIMMODELBUILDER_H
#define PROCESSFLOWPLUGIN_SIM_SIMMODELBUILDER_H

/// @file SimModelBuilder.h
/// @brief processFlow JSON → PlantGraph + 自动 JobTemplate

#include "JobSet.h"
#include "PlantGraph.h"
#include "SimRunConfig.h"

#include <QJsonObject>
#include <QString>

struct SimBuildResult
{
	bool ok = false;
	QString error;
	PlantGraph plant;
	JobSet jobSet;
	double interarrivalSec = 30.0;
};

class SimModelBuilder
{
public:
	static SimBuildResult fromProcessFlowJson(const QJsonObject& flow, const SimRunConfig& config);
};

#endif
