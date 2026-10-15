/// @file SimulationKernel.cpp
/// @brief 统一仿真内核

#include "SimulationKernel.h"

#include <QHash>
#include <algorithm>

namespace
{
std::string laneKey(const SimulationParticipantId& id)
{
	return id.key().toStdString();
}
} // namespace

void SimulationKernel::setIoNetwork(IIoSignalNetwork* network)
{
	m_ioPhase.setNetwork(network);
}

bool SimulationKernel::anyLaneRunning() const
{
	for (const auto& kv : m_lanes)
	{
		if (kv.second && kv.second->isRunning())
		{
			return true;
		}
	}
	return false;
}

void SimulationKernel::setPlaybackRate(double rate)
{
	m_clock.setPlaybackRate(rate);
	for (auto& kv : m_lanes)
	{
		if (RobotLane* rl = dynamic_cast<RobotLane*>(kv.second.get()))
		{
			rl->executor().setPlaybackRate(rate);
		}
	}
}

RobotLane* SimulationKernel::addRobotLane(int instanceIndex)
{
	const SimulationParticipantId id = SimulationParticipantId::robot(instanceIndex);
	const std::string key = laneKey(id);
	const auto it = m_lanes.find(key);
	if (it != m_lanes.end())
	{
		return dynamic_cast<RobotLane*>(it->second.get());
	}
	auto lane = std::make_shared<RobotLane>(instanceIndex);
	RobotLane* raw = lane.get();
	m_lanes.emplace(key, lane);
	return raw;
}

DeviceLane* SimulationKernel::addDeviceLane(const QString& deviceBackendId)
{
	const SimulationParticipantId id = SimulationParticipantId::device(deviceBackendId);
	const std::string key = laneKey(id);
	const auto it = m_lanes.find(key);
	if (it != m_lanes.end())
	{
		DeviceLane* existing = dynamic_cast<DeviceLane*>(it->second.get());
		if (existing)
		{
			existing->setMotionTicker(m_deviceTicker);
		}
		return existing;
	}
	auto lane = std::make_shared<DeviceLane>(deviceBackendId);
	lane->setMotionTicker(m_deviceTicker);
	DeviceLane* raw = lane.get();
	m_lanes.emplace(key, lane);
	return raw;
}

RobotLane* SimulationKernel::robotLane(int instanceIndex) const
{
	const auto it = m_lanes.find(laneKey(SimulationParticipantId::robot(instanceIndex)));
	return it == m_lanes.end() ? nullptr : dynamic_cast<RobotLane*>(it->second.get());
}

DeviceLane* SimulationKernel::deviceLane(const QString& deviceBackendId) const
{
	const auto it = m_lanes.find(laneKey(SimulationParticipantId::device(deviceBackendId)));
	return it == m_lanes.end() ? nullptr : dynamic_cast<DeviceLane*>(it->second.get());
}

RobotProgramExecutor* SimulationKernel::primaryRobotExecutor()
{
	for (auto& kv : m_lanes)
	{
		if (RobotLane* rl = dynamic_cast<RobotLane*>(kv.second.get()))
		{
			if (rl->isRunning())
			{
				return &rl->executor();
			}
		}
	}
	for (auto& kv : m_lanes)
	{
		if (RobotLane* rl = dynamic_cast<RobotLane*>(kv.second.get()))
		{
			return &rl->executor();
		}
	}
	return nullptr;
}

QVector<SimulationParticipantId> SimulationKernel::participantIds() const
{
	QVector<SimulationParticipantId> out;
	for (const auto& kv : m_lanes)
	{
		if (kv.second)
		{
			out.push_back(kv.second->id());
		}
	}
	return out;
}

bool SimulationKernel::startRobotLane(int instanceIndex, IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg,
									  IRobotIoSink* io,
									  const std::vector<std::shared_ptr<RobotInstruction::Base>>& program,
									  const std::vector<RobotInstruction::PlanResult>& motionPlanResults,
									  const QVector<double>& initialJointAnglesRad, QString* errorOut)
{
	if (m_authority.isExternal(instanceIndex))
	{
		if (errorOut)
		{
			*errorOut = QStringLiteral("Robot instance under ExternalController authority.");
		}
		return false;
	}
	RobotLane* lane = addRobotLane(instanceIndex);
	if (!lane)
	{
		return false;
	}
	lane->executor().setPlaybackRate(m_clock.playbackRate());
	return lane->tryStart(doc, osg, io, program, motionPlanResults, initialJointAnglesRad, errorOut);
}

bool SimulationKernel::startDeviceLane(const QString& deviceBackendId,
									   const std::vector<std::shared_ptr<RobotInstruction::Base>>& program,
									   QString* errorOut)
{
	DeviceLane* lane = addDeviceLane(deviceBackendId);
	if (!lane)
	{
		return false;
	}
	lane->setMotionTicker(m_deviceTicker);
	if (IRobotIoSink* sink = m_ioPhase.ensureLatchedSink(deviceBackendId))
	{
		lane->setIoSink(sink);
	}
	return lane->tryStart(program, errorOut);
}

void SimulationKernel::beginSession()
{
	m_sessionActive = true;
	m_clock.reset();
	m_leases.clear();
	m_barriers.clear();
	m_ioPhase.clear();
}

void SimulationKernel::stopLane(const SimulationParticipantId& id)
{
	const auto it = m_lanes.find(laneKey(id));
	if (it == m_lanes.end() || !it->second)
	{
		return;
	}
	it->second->stop();
	m_leases.releaseAll(id);
	m_barriers.leave(id);
	if (!anyLaneRunning())
	{
		m_sessionActive = false;
	}
}

void SimulationKernel::stopAll()
{
	for (auto& kv : m_lanes)
	{
		if (kv.second)
		{
			kv.second->stop();
		}
	}
	m_leases.clear();
	m_barriers.clear();
	if (m_deviceTicker)
	{
		m_deviceTicker->stopAll();
	}
	m_sessionActive = false;
}

void SimulationKernel::collectDiPorts(QHash<QString, QList<int>>& portsByOwner) const
{
	QList<int> ports;
	for (int p = 0; p < 32; ++p)
	{
		ports.append(p);
	}
	for (const auto& kv : m_lanes)
	{
		if (!kv.second)
		{
			continue;
		}
		if (const RobotLane* rl = dynamic_cast<const RobotLane*>(kv.second.get()))
		{
			if (!rl->ioOwnerId().isEmpty())
			{
				portsByOwner.insert(rl->ioOwnerId(), ports);
			}
		}
		else if (kv.second->id().kind == SimulationParticipantKind::Device)
		{
			portsByOwner.insert(kv.second->id().deviceBackendId, ports);
		}
	}
}

SimulationKernelTickResult SimulationKernel::tick(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg,
												  double dtWallSec)
{
	SimulationKernelTickResult result;
	result.sessionActive = m_sessionActive;
	if (!m_sessionActive)
	{
		if (m_deviceTicker && m_deviceTicker->isBusy())
		{
			m_deviceTicker->tick(std::max(0.0, dtWallSec) * m_clock.playbackRate());
		}
		return result;
	}

	const double dtLogic = m_clock.advance(dtWallSec);
	const QVector<SimulationParticipantId> expected = participantIds();

	for (auto& kv : m_lanes)
	{
		ParticipantLane* lane = kv.second.get();
		if (!lane || !lane->isRunning())
		{
			continue;
		}
		if (lane->id().kind == SimulationParticipantKind::Robot &&
			m_authority.isExternal(lane->id().robotInstanceIndex))
		{
			continue;
		}
		(void)lane->step(doc, osg, m_clock.simTimeSec(), m_leases, m_barriers, expected);
		if (lane->state() == ParticipantLaneState::Aborted && !lane->abortSummary().empty())
		{
			result.abortMessages.append(QString::fromStdString(lane->abortSummary()));
		}
	}

	if (m_deviceTicker)
	{
		m_deviceTicker->tick(dtLogic);
	}

	QHash<QString, QList<int>> ports;
	collectDiPorts(ports);
	m_ioPhase.latchAfterPropagate(ports);
	m_ioPhase.swapAll();

	m_applyOrder.applyInOrder(expected, [](const SimulationParticipantId&) {});

	const CollisionGuard::Result cg = m_collisionGuard.evaluate(m_clock.tickIndex());
	if (cg.action == CollisionGuard::Action::StopAll)
	{
		stopAll();
		result.stopAllRequested = true;
		result.abortMessages.append(QStringLiteral("CollisionGuard StopAll"));
	}
	else if (cg.action == CollisionGuard::Action::FreezeLanes)
	{
		for (const SimulationParticipantId& id : cg.freezeIds)
		{
			stopLane(id);
			result.abortMessages.append(QStringLiteral("CollisionGuard FreezeLane %1").arg(id.key()));
		}
	}

	result.anyRunning = anyLaneRunning();
	result.sessionActive = m_sessionActive;
	if (!result.anyRunning)
	{
		m_sessionActive = false;
		result.sessionActive = false;
	}
	return result;
}
