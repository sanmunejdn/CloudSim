#ifndef ROBOTSCENE_SYNCBARRIERREGISTRY_H
#define ROBOTSCENE_SYNCBARRIERREGISTRY_H

/// @file SyncBarrierRegistry.h
/// @brief 多 Lane SyncBarrier：同名到齐放行，超时 Abort

#include "robot_scene_global.h"

#include "SimulationParticipantId.h"

#include <QHash>
#include <QSet>
#include <QString>
#include <QVector>

class ROBOT_SCENE_API SyncBarrierRegistry
{
public:
	void clear();
	void setTimeoutSec(double sec) { m_timeoutSec = sec; }

	/// Lane 到达 barrier；若本拍全体 expected 已到则放行并返回 true
	bool arrive(const QString& name, const SimulationParticipantId& id, const QVector<SimulationParticipantId>& expected,
				double simTimeSec, bool* timedOut);

	bool isWaiting(const SimulationParticipantId& id, QString* outName = nullptr) const;
	void leave(const SimulationParticipantId& id);

private:
	struct BarrierState
	{
		QSet<QString> arrivedKeys;
		double firstArriveSimTime = -1.0;
	};

	QHash<QString, BarrierState> m_barriers;
	QHash<QString, QString> m_waitingByLane; // laneKey -> barrier name
	double m_timeoutSec = 30.0;
};

#endif // ROBOTSCENE_SYNCBARRIERREGISTRY_H
