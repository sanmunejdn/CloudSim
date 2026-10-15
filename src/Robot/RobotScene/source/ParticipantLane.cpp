/// @file ParticipantLane.cpp
/// @brief RobotLane / DeviceLane

#include "ParticipantLane.h"

#include "ResourceLeaseMap.h"
#include "SyncBarrierRegistry.h"

#include <QString>

RobotLane::RobotLane(const int robotInstanceIndex)
	: ParticipantLane(SimulationParticipantId::robot(robotInstanceIndex)), m_instanceIndex(robotInstanceIndex)
{
}

bool RobotLane::tryStart(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, IRobotIoSink* io,
						 const std::vector<std::shared_ptr<RobotInstruction::Base>>& program,
						 const std::vector<RobotInstruction::PlanResult>& motionPlanResults,
						 const QVector<double>& initialJointAnglesRad, QString* errorOut)
{
	m_abortSummary.clear();
	m_waitingBarrier.clear();
	m_executor.setDeviceAxisLeaseFn(nullptr);
	if (!m_executor.tryStart(doc, osg, io, m_instanceIndex, program, motionPlanResults, initialJointAnglesRad, errorOut))
	{
		m_state = ParticipantLaneState::Idle;
		return false;
	}
	m_state = ParticipantLaneState::Running;
	return true;
}

void RobotLane::stop()
{
	m_executor.stop();
	m_state = ParticipantLaneState::Idle;
	m_waitingBarrier.clear();
}

bool RobotLane::isRunning() const
{
	return m_state == ParticipantLaneState::Running || m_state == ParticipantLaneState::WaitingBarrier;
}

const RobotInstruction::Base* RobotLane::currentInstruction() const
{
	return m_executor.currentInstruction();
}

bool RobotLane::step(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, double simTimeSec,
					 ResourceLeaseMap& leases, SyncBarrierRegistry& barriers,
					 const QVector<SimulationParticipantId>& barrierExpected)
{
	if (!isRunning())
	{
		return false;
	}

	m_executor.setDeviceAxisLeaseFn(
		[this, &leases](const std::string& deviceId, int axisIndex, QString* err) -> bool
		{
			const QString key = ResourceLeaseMap::deviceAxisKey(QString::fromStdString(deviceId), axisIndex);
			return leases.tryAcquire(key, m_id, err);
		});

	if (m_executor.inSyncBarrier())
	{
		m_state = ParticipantLaneState::WaitingBarrier;
		m_waitingBarrier = QString::fromStdString(m_executor.syncBarrierName());
		bool timedOut = false;
		const bool released =
			barriers.arrive(m_waitingBarrier, m_id, barrierExpected, simTimeSec, &timedOut);
		if (timedOut)
		{
			m_abortSummary = "SyncBarrier timeout: " + m_executor.syncBarrierName();
			m_executor.stop();
			m_state = ParticipantLaneState::Aborted;
			leases.releaseAll(m_id);
			return false;
		}
		if (released)
		{
			m_executor.clearSyncBarrier();
			m_waitingBarrier.clear();
			m_state = ParticipantLaneState::Running;
		}
		else
		{
			return true;
		}
	}

	const RobotInstructionPlaybackTickResult r = m_executor.tick(doc, osg);
	if (m_executor.inSyncBarrier())
	{
		m_state = ParticipantLaneState::WaitingBarrier;
		m_waitingBarrier = QString::fromStdString(m_executor.syncBarrierName());
		return true;
	}
	if (r == RobotInstructionPlaybackTickResult::Finished)
	{
		m_state = ParticipantLaneState::Finished;
		leases.releaseAll(m_id);
		return false;
	}
	if (r == RobotInstructionPlaybackTickResult::Aborted)
	{
		m_abortSummary = m_executor.lastAbortSummary().empty() ? "RobotLane aborted" : m_executor.lastAbortSummary();
		m_state = ParticipantLaneState::Aborted;
		leases.releaseAll(m_id);
		return false;
	}
	m_state = ParticipantLaneState::Running;
	return true;
}

DeviceLane::DeviceLane(const QString& deviceBackendId)
	: ParticipantLane(SimulationParticipantId::device(deviceBackendId))
{
}

bool DeviceLane::tryStart(const std::vector<std::shared_ptr<RobotInstruction::Base>>& program, QString* errorOut)
{
	if (program.empty())
	{
		if (errorOut)
		{
			*errorOut = QStringLiteral("Empty device program.");
		}
		return false;
	}
	m_steps = program;
	m_pc = 0;
	m_inPose = false;
	m_abortSummary.clear();
	m_waitingBarrier.clear();
	m_state = ParticipantLaneState::Running;
	return true;
}

void DeviceLane::stop()
{
	if (m_ticker && !m_activePoseDevice.empty())
	{
		m_ticker->stopDevice(QString::fromStdString(m_activePoseDevice));
	}
	m_inPose = false;
	m_steps.clear();
	m_pc = 0;
	m_state = ParticipantLaneState::Idle;
	m_waitingBarrier.clear();
}

bool DeviceLane::isRunning() const
{
	return m_state == ParticipantLaneState::Running || m_state == ParticipantLaneState::WaitingBarrier;
}

const RobotInstruction::Base* DeviceLane::currentInstruction() const
{
	if (m_pc == 0 || m_pc > m_steps.size())
	{
		return nullptr;
	}
	return m_steps[m_pc - 1].get();
}

bool DeviceLane::advanceOne(double simTimeSec, ResourceLeaseMap& leases, SyncBarrierRegistry& barriers,
							const QVector<SimulationParticipantId>& barrierExpected)
{
	while (m_pc < m_steps.size())
	{
		const std::shared_ptr<RobotInstruction::Base>& ins = m_steps[m_pc++];
		if (!ins)
		{
			continue;
		}
		switch (ins->type())
		{
		case RobotInstruction::Type::SyncBarrier:
		{
			m_state = ParticipantLaneState::WaitingBarrier;
			m_waitingBarrier = QString::fromStdString(ins->syncBarrierName().empty() ? "cycle" : ins->syncBarrierName());
			bool timedOut = false;
			if (barriers.arrive(m_waitingBarrier, m_id, barrierExpected, simTimeSec, &timedOut))
			{
				m_waitingBarrier.clear();
				m_state = ParticipantLaneState::Running;
				continue;
			}
			if (timedOut)
			{
				m_abortSummary = "SyncBarrier timeout";
				m_state = ParticipantLaneState::Aborted;
				return false;
			}
			--m_pc;
			return true;
		}
		case RobotInstruction::Type::AcquireResource:
		{
			QString err;
			const QString key = QString::fromStdString(ins->leaseResourceKey());
			if (!leases.tryAcquire(key.isEmpty() ? ResourceLeaseMap::deviceKey(m_id.deviceBackendId) : key, m_id, &err))
			{
				m_abortSummary = err.toStdString();
				m_state = ParticipantLaneState::Aborted;
				return false;
			}
			continue;
		}
		case RobotInstruction::Type::ReleaseResource:
		{
			const QString key = QString::fromStdString(ins->leaseResourceKey());
			leases.release(key.isEmpty() ? ResourceLeaseMap::deviceKey(m_id.deviceBackendId) : key, m_id);
			continue;
		}
		case RobotInstruction::Type::SET_DO:
			if (m_io)
			{
				const int port = m_io->resolveNamedPort(ins->ioSignalName(), ins->ioPort());
				m_io->setDigitalOutput(port, ins->ioBoolValue());
			}
			continue;
		case RobotInstruction::Type::GoNamedPose:
		{
			if (!m_ticker)
			{
				m_abortSummary = "No device motion ticker";
				m_state = ParticipantLaneState::Aborted;
				return false;
			}
			QString err;
			const QString deviceId = QString::fromStdString(ins->deviceBackendId().empty()
															   ? m_id.deviceBackendId.toStdString()
															   : ins->deviceBackendId());
			if (!leases.tryAcquire(ResourceLeaseMap::deviceKey(deviceId), m_id, &err))
			{
				m_abortSummary = err.toStdString();
				m_state = ParticipantLaneState::Aborted;
				return false;
			}
			m_activePoseDevice = deviceId.toStdString();
			m_inPose = true;
			if (!m_ticker->startNamedPose(deviceId, QString::fromStdString(ins->namedPoseId()), ins->durationSec()))
			{
				m_abortSummary = "GoNamedPose start failed";
				m_state = ParticipantLaneState::Aborted;
				leases.release(ResourceLeaseMap::deviceKey(deviceId), m_id);
				m_inPose = false;
				return false;
			}
			return true;
		}
		default:
			continue;
		}
	}
	m_state = ParticipantLaneState::Finished;
	leases.releaseAll(m_id);
	return false;
}

bool DeviceLane::step(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, double simTimeSec,
					  ResourceLeaseMap& leases, SyncBarrierRegistry& barriers,
					  const QVector<SimulationParticipantId>& barrierExpected)
{
	(void)doc;
	(void)osg;
	if (!isRunning())
	{
		return false;
	}
	if (m_inPose)
	{
		if (m_ticker && m_ticker->isDeviceBusy(QString::fromStdString(m_activePoseDevice)))
		{
			return true;
		}
		m_inPose = false;
		if (m_writeDone && m_io)
		{
			m_io->setDigitalOutput(0, true);
		}
		leases.release(ResourceLeaseMap::deviceKey(QString::fromStdString(m_activePoseDevice)), m_id);
		m_activePoseDevice.clear();
	}
	if (m_state == ParticipantLaneState::WaitingBarrier)
	{
		bool timedOut = false;
		if (barriers.arrive(m_waitingBarrier, m_id, barrierExpected, simTimeSec, &timedOut))
		{
			++m_pc;
			m_waitingBarrier.clear();
			m_state = ParticipantLaneState::Running;
		}
		else if (timedOut)
		{
			m_abortSummary = "SyncBarrier timeout";
			m_state = ParticipantLaneState::Aborted;
			leases.releaseAll(m_id);
			return false;
		}
		else
		{
			return true;
		}
	}
	return advanceOne(simTimeSec, leases, barriers, barrierExpected);
}
