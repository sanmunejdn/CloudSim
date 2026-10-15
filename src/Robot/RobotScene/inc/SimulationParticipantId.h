#ifndef ROBOTSCENE_SIMULATIONPARTICIPANTID_H
#define ROBOTSCENE_SIMULATIONPARTICIPANTID_H

/// @file SimulationParticipantId.h
/// @brief 仿真参与方标识（机器人实例 / 自定义设备）

#include "robot_scene_global.h"

#include <QHash>
#include <QString>

enum class SimulationParticipantKind
{
	Robot = 0,
	Device
};

struct ROBOT_SCENE_API SimulationParticipantId
{
	SimulationParticipantKind kind = SimulationParticipantKind::Robot;
	int robotInstanceIndex = -1;
	QString deviceBackendId;

	static SimulationParticipantId robot(int instanceIndex)
	{
		SimulationParticipantId id;
		id.kind = SimulationParticipantKind::Robot;
		id.robotInstanceIndex = instanceIndex;
		return id;
	}

	static SimulationParticipantId device(const QString& backendId)
	{
		SimulationParticipantId id;
		id.kind = SimulationParticipantKind::Device;
		id.deviceBackendId = backendId;
		return id;
	}

	bool isValid() const
	{
		if (kind == SimulationParticipantKind::Robot)
		{
			return robotInstanceIndex >= 0;
		}
		return !deviceBackendId.isEmpty();
	}

	QString key() const
	{
		if (kind == SimulationParticipantKind::Robot)
		{
			return QStringLiteral("robot:%1").arg(robotInstanceIndex);
		}
		return QStringLiteral("device:%1").arg(deviceBackendId);
	}

	bool operator==(const SimulationParticipantId& o) const
	{
		return kind == o.kind && robotInstanceIndex == o.robotInstanceIndex && deviceBackendId == o.deviceBackendId;
	}
};

inline uint qHash(const SimulationParticipantId& id, uint seed = 0)
{
	return qHash(id.key(), seed);
}

#endif // ROBOTSCENE_SIMULATIONPARTICIPANTID_H
