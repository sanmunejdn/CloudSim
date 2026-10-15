/// @file WorldApplyOrder.cpp
/// @brief 挂载依赖写回顺序

#include "WorldApplyOrder.h"

#include <QHash>

QVector<SimulationParticipantId> WorldApplyOrder::buildOrder(const QVector<SimulationParticipantId>& participants) const
{
	QHash<QString, QString> deviceToRobotKey;
	for (const WorldApplyMountEdge& e : m_mountEdges)
	{
		if (e.robotInstanceIndex >= 0 && !e.deviceBackendId.isEmpty())
		{
			deviceToRobotKey.insert(e.deviceBackendId, SimulationParticipantId::robot(e.robotInstanceIndex).key());
		}
	}

	QVector<SimulationParticipantId> freeDevices;
	QVector<SimulationParticipantId> robots;
	QHash<QString, QVector<SimulationParticipantId>> mountedByRobot;

	for (const SimulationParticipantId& p : participants)
	{
		if (p.kind == SimulationParticipantKind::Robot)
		{
			robots.push_back(p);
			continue;
		}
		const auto it = deviceToRobotKey.constFind(p.deviceBackendId);
		if (it == deviceToRobotKey.cend())
		{
			freeDevices.push_back(p);
		}
		else
		{
			mountedByRobot[it.value()].push_back(p);
		}
	}

	QVector<SimulationParticipantId> out;
	out += freeDevices;
	for (const SimulationParticipantId& r : robots)
	{
		out.push_back(r);
		out += mountedByRobot.value(r.key());
	}
	return out;
}

void WorldApplyOrder::applyInOrder(const QVector<SimulationParticipantId>& participants, const ApplyFn& apply) const
{
	if (!apply)
	{
		return;
	}
	for (const SimulationParticipantId& p : buildOrder(participants))
	{
		apply(p);
	}
}
