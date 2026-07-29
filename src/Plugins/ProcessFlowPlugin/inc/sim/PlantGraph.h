#ifndef PROCESSFLOWPLUGIN_SIM_PLANTGRAPH_H
#define PROCESSFLOWPLUGIN_SIM_PLANTGRAPH_H

/// @file PlantGraph.h
/// @brief 画布拓扑（设备/缓冲）

#include <QHash>
#include <QString>
#include <QVector>
#include <vector>

struct PlantNode
{
	int id = 0;
	QString kind;
	QString title;
	double cycleTimeSec = 0.0;
	double inventoryQty = 0.0;
	double capacityQty = 0.0;
	double setupTimeSec = 0.0;
	double priority = 0.0;
	double scrapRate = 0.0;
	double batchSize = 1.0;
	double mtbfSec = 0.0;
	double mttrSec = 0.0;
	double requiredInputs = 1.0;
	QString bindingBackendId;
	QString bindingProgramId;
};

struct PlantGraph
{
	QHash<int, PlantNode> nodes;
	QHash<int, QVector<int>> successors;
	int startNodeId = -1;
	int endNodeId = -1;
};

#endif
