/// @file RobotProgramExecutor.cpp
/// @brief 程序执行器

#include "RobotProgramExecutor.h"

#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"
#include "CustomDeviceBackendData.h"
#include "CustomDeviceKinematics.h"
#include "RobotInstructionProgram.h"
#include "RobotSceneKinematics.h"
#include "RunLogger.h"
#include "UrdfRobotLoader.h"

#include "BackendDataManager.h"

#include <QByteArray>
#include <QString>
#include <algorithm>
#include <cmath>
#include <memory>

namespace
{
std::string qToUtf8Std(const QString& s)
{
	const QByteArray utf8 = s.toUtf8();
	return std::string(utf8.constData(), static_cast<size_t>(utf8.size()));
}
} // namespace

void RobotProgramExecutor::stop()
{
	m_running = false;
	m_abortedDueToFailedPlan = false;
	m_lastAbortSummary.clear();
	m_stack.clear();
	m_motionPlanIndex.clear();
	m_motionPlanResults.clear();
	m_jointAnglesRad.clear();
	m_inMotion = false;
	m_activeMotion = nullptr;
	m_segStartJointAngles.clear();
	m_fkMeshWorldT0.clear();
	m_outerWorldAtStart.clear();
	m_simElapsedSec = 0.0;
	m_lastWallSec = 0.0;
	m_segDurationSec = 0.0;
	m_inDeviceAxis = false;
	m_deviceAxisBackendId.clear();
	m_deviceAxisIndex = 0;
	m_deviceAxisQ0 = 0.0;
	m_deviceAxisQ1 = 0.0;
	m_inWaitForSignal = false;
	m_waitCondition = {};
}

void RobotProgramExecutor::setPlaybackRate(double rate)
{
	m_playbackRate = std::clamp(rate, 0.1, 10.0);
}

void RobotProgramExecutor::resetVirtualClock()
{
	m_simElapsedSec = 0.0;
	m_lastWallSec = 0.0;
	m_segmentTimer.restart();
}

void RobotProgramExecutor::advanceVirtualClock()
{
	if (!m_segmentTimer.isValid())
	{
		return;
	}
	const double wallNow = m_segmentTimer.elapsed() / 1000.0;
	const double dt = std::max(0.0, wallNow - m_lastWallSec);
	m_lastWallSec = wallNow;
	m_simElapsedSec += dt * m_playbackRate;
}

bool RobotProgramExecutor::evaluateCondition(const RobotInstruction::Condition& c) const
{
	using RobotInstruction::ConditionKind;
	switch (c.kind)
	{
	case ConditionKind::Always:
		return true;
	case ConditionKind::Never:
		return false;
	case ConditionKind::Io:
		if (m_io)
		{
			bool v = false;
			const int port = m_io->resolveNamedPort(c.signalName, c.ioPort);
			if (m_io->getDigitalInput(port, &v))
			{
				return v == c.ioEquals;
			}
		}
		return c.ioEquals == false;
	case ConditionKind::Compare:
		// Variable table not wired in UI yet; treat unknown left as 0.
		(void)c;
		return false;
	default:
		return true;
	}
}

const RobotInstruction::Base* RobotProgramExecutor::currentInstruction() const
{
	if (m_activeMotion)
	{
		return m_activeMotion;
	}
	if (!m_stack.empty())
	{
		const auto& frame = m_stack.back();
		if (frame.pc > 0 && frame.pc <= frame.steps.size())
		{
			return frame.steps[frame.pc - 1].get();
		}
	}
	return nullptr;
}

double RobotProgramExecutor::motionSegmentProgress01() const
{
	if (!m_inMotion || !m_activeMotion || m_segDurationSec <= 1e-9)
	{
		return 1.0;
	}
	return std::clamp(m_simElapsedSec / m_segDurationSec, 0.0, 1.0);
}

const RobotInstruction::PlanResult* RobotProgramExecutor::planForMotion(const RobotInstruction::Base& ins) const
{
	const auto it = m_motionPlanIndex.find(&ins);
	if (it == m_motionPlanIndex.end() || it->second >= m_motionPlanResults.size())
	{
		return nullptr;
	}
	return &m_motionPlanResults[it->second];
}

const RobotInstruction::PlanResult* RobotProgramExecutor::motionPlanResult(const RobotInstruction::Base* ins) const
{
	if (!ins)
	{
		return nullptr;
	}
	return planForMotion(*ins);
}

bool RobotProgramExecutor::updateMotionPlanResult(const RobotInstruction::Base* ins,
												  const RobotInstruction::PlanResult& plan)
{
	if (!ins)
	{
		return false;
	}
	const auto it = m_motionPlanIndex.find(ins);
	if (it == m_motionPlanIndex.end() || it->second >= m_motionPlanResults.size())
	{
		return false;
	}
	m_motionPlanResults[it->second] = plan;
	return true;
}

bool RobotProgramExecutor::applyJointAngles(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg)
{
	return RobotSceneKinematics::applyJointAnglesFromDocument(doc, osg, m_jointAnglesRad);
}

bool RobotProgramExecutor::startMotionSegment(const RobotInstruction::Base& ins)
{
	const RobotInstruction::PlanResult* plan = planForMotion(ins);
	// 规划失败段：停在此前缀末端，不插值、不跳过（lazyPending 应由 Controller 在 tick 前消掉）
	if (!plan || !plan->ok)
	{
		m_activeMotion = &ins;
		m_inMotion = false;
		m_abortedDueToFailedPlan = true;
		if (plan && plan->plannerName == "lazyPending")
		{
			m_lastAbortSummary = "motion plan still pending";
		}
		else
		{
			m_lastAbortSummary = plan ? plan->summary : "missing motion plan";
		}
		if (m_lastAbortSummary.empty())
		{
			m_lastAbortSummary = "motion planning failed";
		}
		RunLogger::warn(std::string("RobotProgramExecutor: stop before failed motion plan: ") + m_lastAbortSummary);
		return false;
	}

	m_inMotion = true;
	m_activeMotion = &ins;
	m_segStartJointAngles = m_jointAnglesRad;
	m_segDurationSec = 0.5;
	if (plan->durationSec > 1e-6)
	{
		m_segDurationSec = plan->durationSec;
	}
	else
	{
		const auto& ext = ins.extensionProperties();
		const auto it = ext.find("motion.durationSec");
		if (it != ext.end())
		{
			bool ok = false;
			const double v = QString::fromStdString(it->second).toDouble(&ok);
			if (ok && v > 1e-6)
			{
				m_segDurationSec = v;
			}
		}
	}
	m_segDurationSec = std::max(0.05, m_segDurationSec);
	resetVirtualClock();
	return true;
}

bool RobotProgramExecutor::tickMotionSegment(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg)
{
	const RobotInstruction::PlanResult* plan = m_activeMotion ? planForMotion(*m_activeMotion) : nullptr;
	advanceVirtualClock();
	const double u = std::min(1.0, m_simElapsedSec / m_segDurationSec);

	// 多样本轨迹优先于起止关节 lerp（LINE 笛卡尔采样依赖此分支）
	if (plan && plan->ok && plan->jointTrajectoryRad.size() >= 2U)
	{
		const auto& traj = plan->jointTrajectoryRad;
		const size_t waypoints = traj.size() + 1U;
		const double scaled = u * static_cast<double>(waypoints - 1U);
		const size_t i0 = static_cast<size_t>(std::floor(scaled));
		const size_t i1 = std::min(i0 + 1U, waypoints - 1U);
		const double t = scaled - static_cast<double>(i0);
		if (traj.front().size() == static_cast<size_t>(m_jointCount) &&
			m_segStartJointAngles.size() == m_jointAnglesRad.size())
		{
			for (int j = 0; j < m_jointCount; ++j)
			{
				const int gi = m_jointOffset + j;
				const double q0 = (i0 == 0U) ? m_segStartJointAngles[gi]
											 : traj[i0 - 1U][static_cast<size_t>(j)];
				const double q1 = traj[i1 - 1U][static_cast<size_t>(j)];
				m_jointAnglesRad[gi] = q0 + (q1 - q0) * t;
			}
		}
	}
	else if (plan && plan->ok && !plan->jointTargetsRad.empty())
	{
		const size_t n = static_cast<size_t>(m_jointCount);
		if (plan->jointTargetsRad.size() == n && m_segStartJointAngles.size() == m_jointAnglesRad.size())
		{
			for (int j = 0; j < m_jointCount; ++j)
			{
				const int gi = m_jointOffset + j;
				const double q0 = m_segStartJointAngles[gi];
				const double q1 = plan->jointTargetsRad[static_cast<size_t>(j)];
				m_jointAnglesRad[gi] = q0 + (q1 - q0) * u;
			}
		}
	}
	else if (plan && plan->ok && !plan->jointTrajectoryRad.empty())
	{
		const auto& traj = plan->jointTrajectoryRad;
		const double scaled = u * static_cast<double>(traj.size() > 0 ? traj.size() - 1 : 0);
		const size_t i0 = static_cast<size_t>(std::floor(scaled));
		const size_t i1 = std::min(i0 + 1U, traj.empty() ? 0U : traj.size() - 1U);
		const double t = scaled - static_cast<double>(i0);
		if (!traj.empty() && traj[i0].size() == static_cast<size_t>(m_jointCount))
		{
			for (int j = 0; j < m_jointCount; ++j)
			{
				const int gi = m_jointOffset + j;
				const double q0 = (i0 == 0 && m_segStartJointAngles.size() == m_jointAnglesRad.size())
									  ? m_segStartJointAngles[gi]
									  : traj[i0][static_cast<size_t>(j)];
				const double q1 = traj[i1][static_cast<size_t>(j)];
				m_jointAnglesRad[gi] = q0 + (q1 - q0) * t;
			}
		}
	}

	if (!applyJointAngles(doc, osg))
	{
		return false;
	}

	if (u >= 1.0 - 1e-9)
	{
		// 终点强制对齐 jointTargets，避免 LINE 轨迹采样终点与预览/示教分叉
		if (plan && plan->ok && !plan->jointTargetsRad.empty() &&
			plan->jointTargetsRad.size() == static_cast<size_t>(m_jointCount))
		{
			for (int j = 0; j < m_jointCount; ++j)
			{
				m_jointAnglesRad[m_jointOffset + j] = plan->jointTargetsRad[static_cast<size_t>(j)];
			}
			(void)applyJointAngles(doc, osg);
		}
		m_inMotion = false;
		m_activeMotion = nullptr;
	}
	return true;
}

bool RobotProgramExecutor::advanceProgramStep(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg)
{
	(void)doc;
	(void)osg;
	while (!m_stack.empty())
	{
		ListFrame& frame = m_stack.back();
		if (frame.pc >= frame.steps.size())
		{
			if (frame.whileIns != nullptr)
			{
				if (evaluateCondition(frame.whileIns->condition()) && frame.whileIteration < kMaxWhileIterations)
				{
					++frame.whileIteration;
					frame.pc = 0;
					continue;
				}
				if (frame.whileIteration >= kMaxWhileIterations)
				{
					RunLogger::warn("RobotProgramExecutor: While loop iteration limit reached.");
				}
			}
			m_stack.pop_back();
			continue;
		}

		const std::shared_ptr<RobotInstruction::Base> ins = frame.steps[frame.pc++];
		if (!ins)
		{
			continue;
		}

		switch (ins->type())
		{
		case RobotInstruction::Type::PTP:
		case RobotInstruction::Type::LINE:
		case RobotInstruction::Type::ARC:
			return startMotionSegment(*ins);
		case RobotInstruction::Type::WAIT:
		{
			m_inMotion = false;
			m_activeMotion = nullptr;
			m_inDeviceAxis = false;
			const RobotInstruction::Condition& c = ins->condition();
			if (c.kind == RobotInstruction::ConditionKind::Io || c.kind == RobotInstruction::ConditionKind::Compare)
			{
				// 已满足则不阻塞；否则进入等信号（duration=超时，0=无限）
				if (evaluateCondition(c))
				{
					continue;
				}
				m_inWaitForSignal = true;
				m_waitCondition = c;
				m_segDurationSec = std::max(0.0, ins->durationSec());
				resetVirtualClock();
				return true;
			}
			if (c.kind == RobotInstruction::ConditionKind::Never)
			{
				continue;
			}
			m_inWaitForSignal = false;
			m_segDurationSec = std::max(0.0, ins->durationSec());
			resetVirtualClock();
			return true;
		}
		case RobotInstruction::Type::SET_DO:
			if (m_io)
			{
				const int port = m_io->resolveNamedPort(ins->ioSignalName(), ins->ioPort());
				m_io->setDigitalOutput(port, ins->ioBoolValue());
			}
			RunLogger::info(
				qToUtf8Std(QStringLiteral("Set DO port %1 = %2").arg(ins->ioPort()).arg(ins->ioBoolValue())));
			continue;
		case RobotInstruction::Type::SET_AO:
			if (m_io)
			{
				const int port = m_io->resolveNamedPort(ins->ioSignalName(), ins->ioPort());
				m_io->setAnalogOutput(port, ins->ioAnalogValue());
			}
			RunLogger::info(
				qToUtf8Std(QStringLiteral("Set AO port %1 = %2").arg(ins->ioPort()).arg(ins->ioAnalogValue())));
			continue;
		case RobotInstruction::Type::DeviceAxis:
		{
			const std::string deviceId = ins->deviceBackendId();
			const int axisIndex = ins->deviceAxisIndex();
			const double targetQ = ins->deviceAxisTargetQ();
			const double duration = std::max(0.0, ins->durationSec());
			double q0 = targetQ;
			if (BackendDataManager* mgr = doc ? doc->robotBackendManagerForKinematics() : nullptr)
			{
				if (auto data = mgr->getData(deviceId))
				{
					if (auto* device = dynamic_cast<CustomDeviceBackendData*>(data.get()))
					{
						device->ensureQSize();
						const auto& qv = device->qValues();
						if (axisIndex >= 0 && static_cast<size_t>(axisIndex) < qv.size())
						{
							q0 = qv[static_cast<size_t>(axisIndex)];
						}
					}
				}
			}
			if (duration <= 1e-9)
			{
				(void)applyDeviceAxisQ(doc, osg, deviceId, axisIndex, targetQ);
				continue;
			}
			m_inMotion = false;
			m_activeMotion = nullptr;
			m_inDeviceAxis = true;
			m_deviceAxisBackendId = deviceId;
			m_deviceAxisIndex = axisIndex;
			m_deviceAxisQ0 = q0;
			m_deviceAxisQ1 = targetQ;
			m_segDurationSec = duration;
			resetVirtualClock();
			(void)applyDeviceAxisQ(doc, osg, deviceId, axisIndex, q0);
			return true;
		}
		case RobotInstruction::Type::PathPlan:
			continue;
		case RobotInstruction::Type::IF:
		{
			const bool takeThen = evaluateCondition(ins->condition());
			ListFrame branch;
			const auto& steps = takeThen ? ins->nestedSteps() : ins->elseSteps();
			branch.steps.assign(steps.begin(), steps.end());
			if (!branch.steps.empty())
			{
				m_stack.push_back(std::move(branch));
			}
			continue;
		}
		case RobotInstruction::Type::WHILE:
		{
			if (!evaluateCondition(ins->condition()))
			{
				continue;
			}
			ListFrame loop;
			loop.steps.assign(ins->nestedSteps().begin(), ins->nestedSteps().end());
			loop.whileIns = dynamic_cast<const RobotInstruction::WhileInstruction*>(ins.get()); // NOLINT
			loop.whileIteration = 0;
			if (loop.steps.empty())
			{
				continue;
			}
			m_stack.push_back(std::move(loop));
			continue;
		}
		default:
			continue;
		}
	}
	return false;
}

bool RobotProgramExecutor::applyDeviceAxisQ(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg,
											const std::string& deviceId, const int axisIndex, const double q)
{
	if (!doc || deviceId.empty() || axisIndex < 0)
	{
		return false;
	}
	BackendDataManager* mgr = doc->robotBackendManagerForKinematics();
	if (!mgr)
	{
		return false;
	}
	auto data = mgr->getData(deviceId);
	auto* device = data ? dynamic_cast<CustomDeviceBackendData*>(data.get()) : nullptr;
	if (!device)
	{
		RunLogger::warn(std::string("DeviceAxis: custom device not found: ") + deviceId);
		return false;
	}
	device->ensureQSize();
	std::vector<double> qv = device->qValues();
	if (static_cast<size_t>(axisIndex) >= qv.size())
	{
		RunLogger::warn("DeviceAxis: axis index out of range");
		return false;
	}
	qv[static_cast<size_t>(axisIndex)] = q;
	return CustomDeviceKinematics::applyQ(*device, mgr, osg, &qv);
}

bool RobotProgramExecutor::tickWaitForSignal()
{
	if (!m_inWaitForSignal)
	{
		return true;
	}
	if (evaluateCondition(m_waitCondition))
	{
		m_inWaitForSignal = false;
		m_segDurationSec = 0.0;
		return true;
	}
	advanceVirtualClock();
	// durationSec 作为超时；0 表示一直等到信号
	if (m_segDurationSec > 1e-9 && m_simElapsedSec >= m_segDurationSec)
	{
		RunLogger::warn("RobotProgramExecutor: WAIT for signal timed out.");
		m_inWaitForSignal = false;
		m_segDurationSec = 0.0;
		return true;
	}
	return true;
}

bool RobotProgramExecutor::tickDeviceAxisSegment(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg)
{
	if (!m_inDeviceAxis)
	{
		return true;
	}
	advanceVirtualClock();
	double u = 1.0;
	if (m_segDurationSec > 1e-9)
	{
		u = std::clamp(m_simElapsedSec / m_segDurationSec, 0.0, 1.0);
	}
	const double q = m_deviceAxisQ0 + (m_deviceAxisQ1 - m_deviceAxisQ0) * u;
	(void)applyDeviceAxisQ(doc, osg, m_deviceAxisBackendId, m_deviceAxisIndex, q);
	if (u >= 1.0 - 1e-9)
	{
		(void)applyDeviceAxisQ(doc, osg, m_deviceAxisBackendId, m_deviceAxisIndex, m_deviceAxisQ1);
		m_inDeviceAxis = false;
		m_segDurationSec = 0.0;
	}
	return true;
}

bool RobotProgramExecutor::tryStart(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg, IRobotIoSink* io,
									int robotInstanceIndex,
									const std::vector<std::shared_ptr<RobotInstruction::Base>>& program,
									const std::vector<RobotInstruction::PlanResult>& motionPlanResults,
									const QVector<double>& initialJointAnglesRad, QString* errorOut)
{
	stop();
	if (!doc || !osg || !doc->hasRobotSimulationContext())
	{
		if (errorOut)
		{
			*errorOut = QStringLiteral("No document or robot context.");
		}
		return false;
	}
	if (program.empty())
	{
		if (errorOut)
		{
			*errorOut = QStringLiteral("Empty program.");
		}
		return false;
	}

	m_abortedDueToFailedPlan = false;
	m_lastAbortSummary.clear();
	m_io = io;
	m_robotInstanceIndex = robotInstanceIndex;
	m_jointOffset = 0;
	m_jointCount = doc->robotRevoluteJointCountForInstance(robotInstanceIndex);
	for (int i = 0; i < robotInstanceIndex; ++i)
	{
		m_jointOffset += doc->robotRevoluteJointCountForInstance(i);
	}

	const QStringList allJoints = doc->robotRevoluteJointNames();
	m_jointAnglesRad.resize(allJoints.size());
	if (initialJointAnglesRad.size() == allJoints.size())
	{
		m_jointAnglesRad = initialJointAnglesRad;
	}
	else
	{
		m_jointAnglesRad.fill(0.0);
	}

	m_motionPlanResults = motionPlanResults;
	const std::vector<const RobotInstruction::Base*> motions = RobotInstruction::collectMotionInstructions(program);
	m_motionPlanIndex.clear();
	for (size_t i = 0; i < motions.size(); ++i)
	{
		if (motions[i])
		{
			m_motionPlanIndex[motions[i]] = i;
		}
	}
	if (motions.size() != motionPlanResults.size())
	{
		if (errorOut)
		{
			*errorOut = QStringLiteral("Motion plan count does not match motion instructions.");
		}
		stop();
		return false;
	}

	ListFrame root;
	root.steps = program;
	m_stack.push_back(std::move(root));
	m_running = true;
	m_inMotion = false;
	m_inDeviceAxis = false;
	m_inWaitForSignal = false;
	RunLogger::info("RobotProgramExecutor started.");
	return true;
}

RobotInstructionPlaybackTickResult RobotProgramExecutor::tick(IRobotSimulationDocument* doc, IRobotBackendPoseSink* osg)
{
	if (!m_running)
	{
		return RobotInstructionPlaybackTickResult::Continue;
	}
	if (!doc || !osg)
	{
		stop();
		return RobotInstructionPlaybackTickResult::Aborted;
	}

	if (m_inMotion)
	{
		if (!tickMotionSegment(doc, osg))
		{
			stop();
			return RobotInstructionPlaybackTickResult::Aborted;
		}
		if (m_inMotion)
		{
			return RobotInstructionPlaybackTickResult::Continue;
		}
	}

	if (m_inDeviceAxis)
	{
		if (!tickDeviceAxisSegment(doc, osg))
		{
			stop();
			return RobotInstructionPlaybackTickResult::Aborted;
		}
		if (m_inDeviceAxis)
		{
			return RobotInstructionPlaybackTickResult::Continue;
		}
	}

	if (m_inWaitForSignal)
	{
		(void)tickWaitForSignal();
		if (m_inWaitForSignal)
		{
			return RobotInstructionPlaybackTickResult::Continue;
		}
	}

	if (!m_activeMotion && m_segDurationSec > 1e-9 && !m_inMotion && !m_inDeviceAxis && !m_inWaitForSignal &&
		m_segmentTimer.isValid())
	{
		advanceVirtualClock();
		if (m_simElapsedSec < m_segDurationSec)
		{
			return RobotInstructionPlaybackTickResult::Continue;
		}
		m_segDurationSec = 0.0;
	}

	if (advanceProgramStep(doc, osg))
	{
		return RobotInstructionPlaybackTickResult::Continue;
	}

	if (m_abortedDueToFailedPlan)
	{
		m_running = false;
		return RobotInstructionPlaybackTickResult::Aborted;
	}

	m_running = false;
	RunLogger::info("RobotProgramExecutor finished.");
	return RobotInstructionPlaybackTickResult::Finished;
}
