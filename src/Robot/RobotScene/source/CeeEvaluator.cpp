/// @file CeeEvaluator.cpp
/// @brief 薄 CEE 求值

#include "CeeEvaluator.h"

#include "ICeeActionHandler.h"
#include "IIoSignalNetwork.h"
#include "IoPhase.h"
#include "LatchedIoSink.h"
#include "SimRunMode.h"
#include "SimulationKernel.h"
#include "SimulationParticipantId.h"

#include <QStringList>

void CeeEvaluator::setRules(const QVector<CeeRule>& rules)
{
	m_rules = rules;
}

void CeeEvaluator::resetEdgeState()
{
	m_lastDi.clear();
}

bool CeeEvaluator::readDi(IIoSignalNetwork* network, IoPhase& ioPhase, const QString& ownerId, const QString& signal,
						  bool* outValue) const
{
	if (!network || !outValue || ownerId.isEmpty() || signal.isEmpty())
	{
		return false;
	}
	LatchedIoSink* latched = ioPhase.ensureLatchedSink(ownerId);
	IRobotIoSink* sink = latched ? static_cast<IRobotIoSink*>(latched) : network->sink(ownerId);
	if (!sink)
	{
		return false;
	}
	const int port = sink->resolveNamedPort(signal.toStdString(), -1);
	if (port < 0)
	{
		return false;
	}
	return sink->getDigitalInput(port, outValue);
}

void CeeEvaluator::applyAction(const CeeAction& action, IIoSignalNetwork* network, IoPhase& ioPhase,
							   SimulationKernel& kernel, ICeeActionHandler* handler)
{
	switch (action.type)
	{
	case CeeAction::Type::SetDo:
	{
		if (!network || action.ownerId.isEmpty() || action.signal.isEmpty())
		{
			return;
		}
		LatchedIoSink* latched = ioPhase.ensureLatchedSink(action.ownerId);
		IRobotIoSink* sink = latched ? static_cast<IRobotIoSink*>(latched) : network->sink(action.ownerId);
		if (!sink)
		{
			return;
		}
		const int port = sink->resolveNamedPort(action.signal.toStdString(), -1);
		if (port < 0)
		{
			return;
		}
		sink->setDigitalOutput(port, action.value);
		break;
	}
	case CeeAction::Type::StopLane:
	{
		const QString key = action.participant;
		if (key.startsWith(QStringLiteral("robot:")))
		{
			bool ok = false;
			const int idx = key.mid(QStringLiteral("robot:").size()).toInt(&ok);
			if (ok)
			{
				kernel.stopLane(SimulationParticipantId::robot(idx));
			}
		}
		else if (key.startsWith(QStringLiteral("device:")))
		{
			kernel.stopLane(SimulationParticipantId::device(key.mid(QStringLiteral("device:").size())));
		}
		break;
	}
	case CeeAction::Type::GoNamedPose:
		if (handler)
		{
			QString err;
			(void)handler->onGoNamedPose(action.deviceId, action.poseId, action.durationSec, &err);
		}
		break;
	case CeeAction::Type::StartRobotProgram:
		if (handler)
		{
			QString err;
			(void)handler->onStartRobotProgram(action.instanceIndex, action.programId, &err);
		}
		break;
	}
}

void CeeEvaluator::evaluate(IIoSignalNetwork* network, IoPhase& ioPhase, SimulationKernel& kernel,
							ICeeActionHandler* handler)
{
	if (!network)
	{
		return;
	}
	for (const CeeRule& rule : m_rules)
	{
		if (!rule.enabled || rule.when.ownerId.isEmpty() || rule.when.signal.isEmpty())
		{
			continue;
		}
		bool now = false;
		if (!readDi(network, ioPhase, rule.when.ownerId, rule.when.signal, &now))
		{
			continue;
		}
		const QString edgeKey = rule.when.ownerId + QLatin1Char('|') + rule.when.signal;
		const bool had = m_lastDi.contains(edgeKey);
		const bool prev = had ? m_lastDi.value(edgeKey) : now;
		m_lastDi.insert(edgeKey, now);

		bool fire = false;
		switch (rule.when.type)
		{
		case CeeCondition::Type::RisingEdge:
			// 首拍仅建记忆，避免加载瞬间误触发
			fire = had && !prev && now;
			break;
		case CeeCondition::Type::LevelHigh:
			fire = now;
			break;
		case CeeCondition::Type::LevelLow:
			fire = !now;
			break;
		}
		if (!fire)
		{
			continue;
		}
		for (const CeeAction& a : rule.actions)
		{
			applyAction(a, network, ioPhase, kernel, handler);
		}
	}
}

namespace
{
class FakeSink final : public IRobotIoSink
{
public:
	bool di = false;
	bool lastDo = false;
	int lastDoPort = -1;

	bool getDigitalInput(int port, bool* outValue) const override
	{
		(void)port;
		if (outValue)
		{
			*outValue = di;
		}
		return true;
	}
	void setDigitalOutput(int port, bool value) override
	{
		lastDoPort = port;
		lastDo = value;
	}
	void setAnalogOutput(int /*port*/, double /*value*/) override {}
	int resolveNamedPort(const std::string& signalName, int fallbackPort) const override
	{
		if (signalName == "Start")
		{
			return 1;
		}
		return fallbackPort;
	}
};

class FakeNet final : public IIoSignalNetwork
{
public:
	FakeSink sinkObj;
	QStringList ownerIds() const override { return {QStringLiteral("devA")}; }
	IRobotIoSink* sink(const QString& ownerId) override
	{
		return ownerId == QStringLiteral("devA") ? &sinkObj : nullptr;
	}
	void propagateFrom(const QString&) override {}
	void propagateAll() override {}
};

class CountingHandler final : public ICeeActionHandler
{
public:
	int poseStarts = 0;
	bool onGoNamedPose(const QString&, const QString&, double, QString*) override
	{
		++poseStarts;
		return true;
	}
	bool onStartRobotProgram(int, const QString&, QString*) override { return true; }
};
} // namespace

bool runCeeEvaluatorSelfTest(QString* errOut)
{
	FakeNet net;
	IoPhase phase;
	phase.setNetwork(&net);
	SimulationKernel kernel;
	kernel.setIoNetwork(&net);
	kernel.setRunMode(SimRunMode::EventBased);

	CeeRule rule;
	rule.id = QStringLiteral("t1");
	rule.when.type = CeeCondition::Type::RisingEdge;
	rule.when.ownerId = QStringLiteral("devA");
	rule.when.signal = QStringLiteral("Start");
	CeeAction act;
	act.type = CeeAction::Type::GoNamedPose;
	act.deviceId = QStringLiteral("devA");
	act.poseId = QStringLiteral("p1");
	rule.actions.push_back(act);

	CeeEvaluator eval;
	eval.setRules({rule});
	CountingHandler handler;

	auto latchSample = [&]()
	{
		phase.ensureLatchedSink(QStringLiteral("devA"));
		QHash<QString, QList<int>> ports;
		ports.insert(QStringLiteral("devA"), QList<int>{1});
		phase.latchAfterPropagate(ports);
		phase.swapAll();
	};

	net.sinkObj.di = false;
	latchSample();
	eval.evaluate(&net, phase, kernel, &handler);
	if (handler.poseStarts != 0)
	{
		if (errOut)
		{
			*errOut = QStringLiteral("risingEdge must not fire on first sample");
		}
		return false;
	}

	net.sinkObj.di = true;
	latchSample();
	eval.evaluate(&net, phase, kernel, &handler);
	if (handler.poseStarts != 1)
	{
		if (errOut)
		{
			*errOut = QStringLiteral("risingEdge should fire once");
		}
		return false;
	}

	latchSample();
	eval.evaluate(&net, phase, kernel, &handler);
	if (handler.poseStarts != 1)
	{
		if (errOut)
		{
			*errOut = QStringLiteral("risingEdge must not retrigger while high");
		}
		return false;
	}

	CeeRule levelRule = rule;
	levelRule.id = QStringLiteral("t2");
	levelRule.when.type = CeeCondition::Type::LevelHigh;
	CeeAction setDo;
	setDo.type = CeeAction::Type::SetDo;
	setDo.ownerId = QStringLiteral("devA");
	setDo.signal = QStringLiteral("Start");
	setDo.value = true;
	levelRule.actions = {setDo};
	eval.setRules({levelRule});
	eval.resetEdgeState();
	net.sinkObj.di = true;
	latchSample();
	eval.evaluate(&net, phase, kernel, &handler);
	if (!net.sinkObj.lastDo || net.sinkObj.lastDoPort != 1)
	{
		if (errOut)
		{
			*errOut = QStringLiteral("levelHigh setDo failed");
		}
		return false;
	}
	return true;
}
