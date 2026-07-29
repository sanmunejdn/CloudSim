/// @file SimModelBuilder.cpp
/// @brief 从图 JSON 构建 PlantGraph 与自动工艺路径

#include "sim/SimModelBuilder.h"

#include "ProcessFlowNodeProps.h"

#include <QJsonArray>
#include <QQueue>
#include <algorithm>
#include <limits>

namespace
{
bool isMachineKind(const QString& kind)
{
	return ProcessFlowNodeProps::isMachineKind(kind);
}

bool isBufferKind(const QString& kind)
{
	return ProcessFlowNodeProps::isBufferKind(kind);
}

QVector<int> findPath(const PlantGraph& plant, int from, int to)
{
	if (from < 0 || to < 0 || !plant.nodes.contains(from) || !plant.nodes.contains(to))
	{
		return {};
	}
	if (from == to)
	{
		return {from};
	}
	QHash<int, int> parent;
	QQueue<int> q;
	q.enqueue(from);
	parent.insert(from, -1);
	while (!q.isEmpty())
	{
		const int u = q.dequeue();
		const QVector<int> next = plant.successors.value(u);
		for (int v : next)
		{
			if (parent.contains(v))
			{
				continue;
			}
			parent.insert(v, u);
			if (v == to)
			{
				QVector<int> path;
				for (int x = to; x >= 0; x = parent.value(x))
				{
					path.prepend(x);
					if (x == from)
					{
						break;
					}
				}
				return path;
			}
			q.enqueue(v);
		}
	}
	return {};
}

double bufferCapacityOnPath(const PlantGraph& plant, int fromMachine, int toMachine)
{
	const QVector<int> path = findPath(plant, fromMachine, toMachine);
	if (path.size() < 2)
	{
		return -1.0;
	}
	double cap = std::numeric_limits<double>::infinity();
	bool any = false;
	for (int i = 1; i + 1 < path.size(); ++i)
	{
		const PlantNode& n = plant.nodes.value(path[i]);
		if (!isBufferKind(n.kind))
		{
			continue;
		}
		any = true;
		cap = std::min(cap, std::max(0.0, n.capacityQty));
	}
	if (!any)
	{
		return -1.0;
	}
	return cap;
}
} // namespace

SimBuildResult SimModelBuilder::fromProcessFlowJson(const QJsonObject& flow, const SimRunConfig& config)
{
	SimBuildResult result;
	PlantGraph& plant = result.plant;

	const QJsonArray nodes = flow.value(QStringLiteral("nodes")).toArray();
	const QJsonArray edges = flow.value(QStringLiteral("edges")).toArray();
	if (nodes.isEmpty())
	{
		result.error = QStringLiteral("processFlow has no nodes");
		return result;
	}

	for (const QJsonValue& v : nodes)
	{
		const QJsonObject o = v.toObject();
		PlantNode n;
		n.id = o.value(QStringLiteral("id")).toInt();
		n.title = o.value(QStringLiteral("title")).toString();
		const QJsonObject props = o.value(QStringLiteral("props")).toObject();
		n.kind = props.value(QStringLiteral("kind")).toString(QStringLiteral("station"));
		n.cycleTimeSec = props.value(QStringLiteral("cycleTimeSec")).toDouble();
		n.inventoryQty = props.value(QStringLiteral("inventoryQty")).toDouble();
		n.capacityQty = props.value(QStringLiteral("capacityQty")).toDouble();
		n.setupTimeSec = props.value(QStringLiteral("setupTimeSec")).toDouble();
		n.priority = props.value(QStringLiteral("priority")).toDouble();
		n.scrapRate = props.value(QStringLiteral("scrapRate")).toDouble();
		n.batchSize = props.value(QStringLiteral("batchSize")).toDouble(1.0);
		n.mtbfSec = props.value(QStringLiteral("mtbfSec")).toDouble();
		n.mttrSec = props.value(QStringLiteral("mttrSec")).toDouble();
		n.requiredInputs = props.value(QStringLiteral("requiredInputs")).toDouble(1.0);
		const QJsonObject binding = props.value(QStringLiteral("binding")).toObject();
		n.bindingBackendId = binding.value(QStringLiteral("backendId")).toString();
		n.bindingProgramId = binding.value(QStringLiteral("programId")).toString();
		plant.nodes.insert(n.id, n);
		if (n.kind == QStringLiteral("start") && plant.startNodeId < 0)
		{
			plant.startNodeId = n.id;
		}
		if (n.kind == QStringLiteral("end") && plant.endNodeId < 0)
		{
			plant.endNodeId = n.id;
		}
	}

	for (const QJsonValue& v : edges)
	{
		const QJsonObject o = v.toObject();
		const int from = o.value(QStringLiteral("from")).toInt();
		const int to = o.value(QStringLiteral("to")).toInt();
		if (!plant.nodes.contains(from) || !plant.nodes.contains(to))
		{
			continue;
		}
		plant.successors[from].append(to);
	}

	if (plant.startNodeId < 0 || plant.endNodeId < 0)
	{
		result.error = QStringLiteral("need at least one start and one end node");
		return result;
	}

	const QVector<int> path = findPath(plant, plant.startNodeId, plant.endNodeId);
	if (path.isEmpty())
	{
		result.error = QStringLiteral("no path from start to end");
		return result;
	}

	const QJsonObject jobSetObj = flow.value(QStringLiteral("jobSet")).toObject();
	const QJsonArray templatesArr = jobSetObj.value(QStringLiteral("templates")).toArray();
	if (!templatesArr.isEmpty())
	{
		for (const QJsonValue& tv : templatesArr)
		{
			const QJsonObject to = tv.toObject();
			JobTemplate tmpl;
			tmpl.name = to.value(QStringLiteral("name")).toString(QStringLiteral("job"));
			const QJsonArray opsArr = to.value(QStringLiteral("ops")).toArray();
			for (const QJsonValue& ov : opsArr)
			{
				const QJsonObject oo = ov.toObject();
				OpSpec op;
				op.machineNodeId = oo.value(QStringLiteral("machineNodeId")).toInt(-1);
				op.processTimeSec = std::max(0.0, oo.value(QStringLiteral("processTimeSec")).toDouble());
				op.setupTimeSec = std::max(0.0, oo.value(QStringLiteral("setupTimeSec")).toDouble());
				op.priority = oo.value(QStringLiteral("priority")).toDouble();
				op.scrapRate = std::clamp(oo.value(QStringLiteral("scrapRate")).toDouble(0.0), 0.0, 1.0);
				if (!plant.nodes.contains(op.machineNodeId) || !isMachineKind(plant.nodes.value(op.machineNodeId).kind))
				{
					result.error = QStringLiteral("jobSet op references invalid machine");
					return result;
				}
				const PlantNode& mn = plant.nodes.value(op.machineNodeId);
				if (!oo.contains(QStringLiteral("setupTimeSec")))
				{
					op.setupTimeSec = mn.setupTimeSec;
				}
				if (!oo.contains(QStringLiteral("priority")))
				{
					op.priority = mn.priority;
				}
				if (!oo.contains(QStringLiteral("scrapRate")))
				{
					op.scrapRate = mn.scrapRate;
				}
				tmpl.ops.append(op);
			}
			if (tmpl.ops.isEmpty())
			{
				continue;
			}
			for (int i = 0; i + 1 < tmpl.ops.size(); ++i)
			{
				tmpl.ops[i].interBufferCapacity =
					bufferCapacityOnPath(plant, tmpl.ops[i].machineNodeId, tmpl.ops[i + 1].machineNodeId);
			}
			result.jobSet.templates.append(tmpl);
		}
	}

	if (result.jobSet.templates.isEmpty())
	{
		JobTemplate tmpl;
		tmpl.name = QStringLiteral("auto");
		for (int id : path)
		{
			const PlantNode& n = plant.nodes.value(id);
			if (!isMachineKind(n.kind))
			{
				continue;
			}
			OpSpec op;
			op.machineNodeId = id;
			op.processTimeSec = std::max(0.0, n.cycleTimeSec);
			op.setupTimeSec = std::max(0.0, n.setupTimeSec);
			op.priority = n.priority;
			op.scrapRate = (n.kind == QStringLiteral("inspect")) ? std::clamp(n.scrapRate, 0.0, 1.0) : 0.0;
			op.batchSize = std::max(1.0, n.batchSize);
			op.mtbfSec = n.mtbfSec;
			op.mttrSec = n.mttrSec;
			op.requiredInputs = std::max(1.0, n.requiredInputs);
			op.kind = n.kind;
			tmpl.ops.append(op);
		}
		if (tmpl.ops.isEmpty())
		{
			result.error = QStringLiteral("path has no station/inspect");
			return result;
		}
		for (int i = 0; i + 1 < tmpl.ops.size(); ++i)
		{
			tmpl.ops[i].interBufferCapacity =
				bufferCapacityOnPath(plant, tmpl.ops[i].machineNodeId, tmpl.ops[i + 1].machineNodeId);
		}
		result.jobSet.templates.append(tmpl);
	}

	const PlantNode& start = plant.nodes.value(plant.startNodeId);
	result.interarrivalSec = start.cycleTimeSec > 1e-9 ? start.cycleTimeSec : config.defaultInterarrivalSec;
	result.ok = true;
	return result;
}
