#ifndef ROBOTSCENE_PARTICIPANTLANE_H
#define ROBOTSCENE_PARTICIPANTLANE_H

/// @file ParticipantLane.h
/// @brief 仿真 Lane：机器人 Executor / 设备程序状态机

#include "robot_scene_global.h"

#include "IDeviceMotionTicker.h"
#include "RobotInstructionModel.h"
#include "RobotInstructionPlaybackEngine.h"
#include "RobotProgramExecutor.h"
#include "SimulationParticipantId.h"

#include <QString>
#include <memory>
#include <string>
#include <vector>

class IRobotSimulationDocument;
class IRobotBackendPoseSink;
class IRobotIoSink;
class ResourceLeaseMap;
class SyncBarrierRegistry;

enum class ROBOT_SCENE_API ParticipantLaneState
{
	Idle = 0,
	Running,
	WaitingBarrier,
	Aborted,
	Finished
};

class ROBOT_SCENE_API ParticipantLane
{
public:
	virtual ~ParticipantLane() = default;

	const SimulationParticipantId& id() const { return m_id; }
	ParticipantLaneState state() const { return m_state; }
	const std::string& abortSummary() const { return m_abortSummary; }
	QString waitingBarrierName() const { return m_waitingBarrier; }

	virtual void stop() = 0;
	virtual bool isRunning() const = 0;
	/// 返回 true 表示本拍仍活跃（Running/WaitingBarrier）
	virtual bool step(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, double simTimeSec,
					  ResourceLeaseMap& leases, SyncBarrierRegistry& barriers,
					  const QVector<SimulationParticipantId>& barrierExpected) = 0;

	virtual const RobotInstruction::Base* currentInstruction() const { return nullptr; }
	virtual RobotProgramExecutor* robotExecutor() { return nullptr; }

protected:
	explicit ParticipantLane(SimulationParticipantId id) : m_id(std::move(id)) {}

	SimulationParticipantId m_id;
	ParticipantLaneState m_state = ParticipantLaneState::Idle;
	std::string m_abortSummary;
	QString m_waitingBarrier;
};

class ROBOT_SCENE_API RobotLane final : public ParticipantLane
{
public:
	explicit RobotLane(int robotInstanceIndex);

	void setIoOwnerId(const QString& ownerId) { m_ioOwnerId = ownerId; }
	QString ioOwnerId() const { return m_ioOwnerId; }

	RobotProgramExecutor& executor() { return m_executor; }
	const RobotProgramExecutor& executor() const { return m_executor; }
	RobotProgramExecutor* robotExecutor() override { return &m_executor; }

	bool tryStart(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, IRobotIoSink* io,
				  const std::vector<std::shared_ptr<RobotInstruction::Base>>& program,
				  const std::vector<RobotInstruction::PlanResult>& motionPlanResults,
				  const QVector<double>& initialJointAnglesRad, QString* errorOut);

	void stop() override;
	bool isRunning() const override;
	bool step(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, double simTimeSec, ResourceLeaseMap& leases,
			  SyncBarrierRegistry& barriers, const QVector<SimulationParticipantId>& barrierExpected) override;
	const RobotInstruction::Base* currentInstruction() const override;

private:
	bool handleKernelOpcodes(double simTimeSec, ResourceLeaseMap& leases, SyncBarrierRegistry& barriers,
							 const QVector<SimulationParticipantId>& barrierExpected);

	RobotProgramExecutor m_executor;
	int m_instanceIndex = 0;
	QString m_ioOwnerId;
};

class ROBOT_SCENE_API DeviceLane final : public ParticipantLane
{
public:
	explicit DeviceLane(const QString& deviceBackendId);

	void setMotionTicker(IDeviceMotionTicker* ticker) { m_ticker = ticker; }
	void setIoSink(IRobotIoSink* io) { m_io = io; }

	bool tryStart(const std::vector<std::shared_ptr<RobotInstruction::Base>>& program, QString* errorOut);
	void stop() override;
	bool isRunning() const override;
	bool step(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, double simTimeSec, ResourceLeaseMap& leases,
			  SyncBarrierRegistry& barriers, const QVector<SimulationParticipantId>& barrierExpected) override;
	const RobotInstruction::Base* currentInstruction() const override;

	/// 姿态结束写 Done DO（端口 0 默认）
	void setWriteDoneOnPoseFinished(bool on) { m_writeDone = on; }

private:
	bool advanceOne(double simTimeSec, ResourceLeaseMap& leases, SyncBarrierRegistry& barriers,
					const QVector<SimulationParticipantId>& barrierExpected);

	IDeviceMotionTicker* m_ticker = nullptr;
	IRobotIoSink* m_io = nullptr;
	std::vector<std::shared_ptr<RobotInstruction::Base>> m_steps;
	size_t m_pc = 0;
	bool m_inPose = false;
	bool m_writeDone = true;
	std::string m_activePoseDevice;
};

#endif // ROBOTSCENE_PARTICIPANTLANE_H
