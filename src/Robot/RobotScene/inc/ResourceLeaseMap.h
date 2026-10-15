#ifndef ROBOTSCENE_RESOURCELEASEMAP_H
#define ROBOTSCENE_RESOURCELEASEMAP_H

/// @file ResourceLeaseMap.h
/// @brief 仿真写权租约；冲突时调用方 Abort Lane

#include "robot_scene_global.h"

#include "SimulationParticipantId.h"

#include <QHash>
#include <QString>
#include <QStringList>

class ROBOT_SCENE_API ResourceLeaseMap
{
public:
	static QString deviceKey(const QString& backendId) { return QStringLiteral("device:%1").arg(backendId); }
	static QString deviceAxisKey(const QString& backendId, int axisIndex)
	{
		return QStringLiteral("deviceAxis:%1:%2").arg(backendId).arg(axisIndex);
	}
	static QString extAxisKey(int robotInstance, int axisIndex)
	{
		return QStringLiteral("extAxis:%1:%2").arg(robotInstance).arg(axisIndex);
	}
	static QString workpieceKey(const QString& backendId) { return QStringLiteral("workpiece:%1").arg(backendId); }

	void clear();

	/// 已持有同一 owner 视为成功；被其它 owner 占用返回 false
	bool tryAcquire(const QString& resourceKey, const SimulationParticipantId& owner, QString* errOut = nullptr);
	void release(const QString& resourceKey, const SimulationParticipantId& owner);
	void releaseAll(const SimulationParticipantId& owner);

	SimulationParticipantId ownerOf(const QString& resourceKey) const;
	bool isHeldBy(const QString& resourceKey, const SimulationParticipantId& owner) const;

private:
	QHash<QString, SimulationParticipantId> m_holders;
};

#endif // ROBOTSCENE_RESOURCELEASEMAP_H
