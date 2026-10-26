#ifndef ROBOTSCENE_SIMULATIONKERNEL_H
#define ROBOTSCENE_SIMULATIONKERNEL_H

/// @file SimulationKernel.h
/// @brief 多 Lane 统一仿真内核（逻辑时钟 / IO 下一拍 / 租约 / Barrier / Apply / 碰撞）

#include "robot_scene_global.h"

#include "CeeEvaluator.h"
#include "CeeRule.h"
#include "CollisionGuard.h"
#include "ControlAuthority.h"
#include "EventLogicAuthority.h"
#include "IDeviceMotionTicker.h"
#include "IIoSignalNetwork.h"
#include "IoPhase.h"
#include "LogicalClock.h"
#include "ParticipantLane.h"
#include "ResourceLeaseMap.h"
#include "SimRunMode.h"
#include "SyncBarrierRegistry.h"
#include "WorldApplyOrder.h"

#include <QHash>
#include <QList>
#include <QString>
#include <QStringList>
#include <QVector>
#include <memory>
#include <unordered_map>

class ICeeActionHandler;
class IRobotSimulationDocument;
class IRobotBackendPoseSink;

struct ROBOT_SCENE_API SimulationKernelTickResult
{
	bool sessionActive = false;
	bool anyRunning = false;
	bool stopAllRequested = false;
	QStringList abortMessages;
};

class ROBOT_SCENE_API SimulationKernel
{
public:
	LogicalClock& clock() { return m_clock; }
	const LogicalClock& clock() const { return m_clock; }
	ResourceLeaseMap& leases() { return m_leases; }
	IoPhase& ioPhase() { return m_ioPhase; }
	WorldApplyOrder& applyOrder() { return m_applyOrder; }
	ControlAuthorityMap& authority() { return m_authority; }
	CollisionGuard& collisionGuard() { return m_collisionGuard; }
	SyncBarrierRegistry& barriers() { return m_barriers; }

	void setIoNetwork(IIoSignalNetwork* network);
	void setDeviceMotionTicker(IDeviceMotionTicker* ticker) { m_deviceTicker = ticker; }
	void setCeeActionHandler(ICeeActionHandler* handler) { m_ceeHandler = handler; }

	void setRunMode(SimRunMode mode) { m_runMode = mode; }
	SimRunMode runMode() const { return m_runMode; }
	void setEventLogicAuthority(EventLogicAuthority authority) { m_eventLogicAuthority = authority; }
	EventLogicAuthority eventLogicAuthority() const { return m_eventLogicAuthority; }
	void setCeeRules(const QVector<CeeRule>& rules);
	const QVector<CeeRule>& ceeRules() const { return m_cee.rules(); }
	CeeEvaluator& ceeEvaluator() { return m_cee; }

	bool isSessionActive() const { return m_sessionActive; }
	bool anyLaneRunning() const;

	// 时间轴等空窗场景：无 Lane 时仍保持 session，避免首拍/片段间隙被 TimeBased 收尾
	void setKeepSessionAliveWithoutLanes(bool keep) { m_keepSessionAliveWithoutLanes = keep; }
	bool keepSessionAliveWithoutLanes() const { return m_keepSessionAliveWithoutLanes; }

	void setPlaybackRate(double rate);
	double playbackRate() const { return m_clock.playbackRate(); }

	RobotLane* addRobotLane(int instanceIndex);
	DeviceLane* addDeviceLane(const QString& deviceBackendId);
	RobotLane* robotLane(int instanceIndex) const;
	DeviceLane* deviceLane(const QString& deviceBackendId) const;
	RobotProgramExecutor* primaryRobotExecutor();
	QVector<SimulationParticipantId> participantIds() const;

	bool startRobotLane(int instanceIndex, IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, IRobotIoSink* io,
						const std::vector<std::shared_ptr<RobotInstruction::Base>>& program,
						const std::vector<RobotInstruction::PlanResult>& motionPlanResults,
						const QVector<double>& initialJointAnglesRad, QString* errorOut);
	bool startDeviceLane(const QString& deviceBackendId,
						 const std::vector<std::shared_ptr<RobotInstruction::Base>>& program, QString* errorOut);

	void beginSession();
	void stopLane(const SimulationParticipantId& id);
	void stopAll();

	SimulationKernelTickResult tick(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, double dtWallSec);

private:
	void collectDiPorts(QHash<QString, QList<int>>& portsByOwner) const;

	bool m_sessionActive = false;
	bool m_keepSessionAliveWithoutLanes = false;
	SimRunMode m_runMode = SimRunMode::TimeBased;
	EventLogicAuthority m_eventLogicAuthority = EventLogicAuthority::LocalCee;
	LogicalClock m_clock;
	ResourceLeaseMap m_leases;
	IoPhase m_ioPhase;
	WorldApplyOrder m_applyOrder;
	ControlAuthorityMap m_authority;
	CollisionGuard m_collisionGuard;
	SyncBarrierRegistry m_barriers;
	CeeEvaluator m_cee;
	IDeviceMotionTicker* m_deviceTicker = nullptr;
	ICeeActionHandler* m_ceeHandler = nullptr;
	IIoSignalNetwork* m_ioNetwork = nullptr;
	std::unordered_map<std::string, std::shared_ptr<ParticipantLane>> m_lanes;
};

#endif // ROBOTSCENE_SIMULATIONKERNEL_H
