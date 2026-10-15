#ifndef ROBOTSCENE_WORLDAPPLYORDER_H
#define ROBOTSCENE_WORLDAPPLYORDER_H

/// @file WorldApplyOrder.h
/// @brief 按挂载依赖排序的场景写回

#include "robot_scene_global.h"

#include "SimulationParticipantId.h"

#include <QString>
#include <QStringList>
#include <QVector>
#include <functional>

class IRobotSimulationDocument;
class IRobotBackendPoseSink;

struct ROBOT_SCENE_API WorldApplyMountEdge
{
	int robotInstanceIndex = -1;
	QString deviceBackendId;
};

class ROBOT_SCENE_API WorldApplyOrder
{
public:
	void setMountEdges(QVector<WorldApplyMountEdge> edges) { m_mountEdges = std::move(edges); }
	const QVector<WorldApplyMountEdge>& mountEdges() const { return m_mountEdges; }

	/// 拓扑序：独立设备 → 机器人 → 挂载在该机器人上的设备
	QVector<SimulationParticipantId> buildOrder(const QVector<SimulationParticipantId>& participants) const;

	using ApplyFn = std::function<void(const SimulationParticipantId&)>;
	void applyInOrder(const QVector<SimulationParticipantId>& participants, const ApplyFn& apply) const;

private:
	QVector<WorldApplyMountEdge> m_mountEdges;
};

#endif // ROBOTSCENE_WORLDAPPLYORDER_H
