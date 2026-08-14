/// @file NamedSignalIoSink.cpp
/// @brief NamedSignalIoSink 实现

#include "NamedSignalIoSink.h"

#include "RunLogger.h"

NamedSignalIoSink::NamedSignalIoSink(QObject* parent) : QObject(parent) {}

void NamedSignalIoSink::setSignalTable(RobotIo::NamedSignalTable* table)
{
	m_table = table;
}

void NamedSignalIoSink::setIoSinkBackend(const RobotIoSinkBackend backend)
{
	m_backend = backend;
	// PlcStub：真机 Bridge/PlcComm 适配器接线点（本期不接硬件）
	if (backend == RobotIoSinkBackend::PlcStub)
	{
		RunLogger::info("NamedSignalIoSink: PlcStub backend selected (no hardware wired).");
	}
}

void NamedSignalIoSink::resetRuntimeFromTable(const bool keepForcedDi)
{
	m_di.clear();
	m_do.clear();
	m_ai.clear();
	m_ao.clear();
	if (!keepForcedDi)
	{
		m_diForced.clear();
	}
	if (!m_table)
	{
		emit ioValuesChanged();
		return;
	}
	for (const RobotIo::SignalDef& s : m_table->entries())
	{
		switch (s.kind)
		{
		case RobotIo::SignalKind::DI:
			m_di[s.port] = s.defaultBool;
			break;
		case RobotIo::SignalKind::DO:
			m_do[s.port] = s.defaultBool;
			break;
		case RobotIo::SignalKind::AI:
			m_ai[s.port] = s.defaultAnalog;
			break;
		case RobotIo::SignalKind::AO:
			m_ao[s.port] = s.defaultAnalog;
			break;
		}
	}
	emit ioValuesChanged();
}

void NamedSignalIoSink::setDigitalOutput(const int port, const bool value)
{
	m_do[port] = value;
	RunLogger::info(std::string("Simulation IO: DO ") + std::to_string(port) + " = " + (value ? "1" : "0"));
	emit ioValuesChanged();
}

void NamedSignalIoSink::setAnalogOutput(const int port, const double value)
{
	m_ao[port] = value;
	RunLogger::info(std::string("Simulation IO: AO ") + std::to_string(port) + " = " + std::to_string(value));
	emit ioValuesChanged();
}

bool NamedSignalIoSink::getDigitalInput(const int port, bool* outValue) const
{
	if (!outValue)
	{
		return false;
	}
	const auto forced = m_diForced.constFind(port);
	if (forced != m_diForced.constEnd())
	{
		*outValue = forced.value();
		return true;
	}
	const auto it = m_di.constFind(port);
	if (it != m_di.constEnd())
	{
		*outValue = it.value();
		return true;
	}
	*outValue = false;
	return true;
}

int NamedSignalIoSink::resolveNamedPort(const std::string& signalName, const int fallbackPort) const
{
	if (m_table)
	{
		return m_table->resolvePort(signalName, fallbackPort);
	}
	return fallbackPort;
}

bool NamedSignalIoSink::getDigitalOutput(const int port, bool* outValue) const
{
	if (!outValue)
	{
		return false;
	}
	const auto it = m_do.constFind(port);
	if (it != m_do.constEnd())
	{
		*outValue = it.value();
		return true;
	}
	*outValue = false;
	return true;
}

bool NamedSignalIoSink::getAnalogOutput(const int port, double* outValue) const
{
	if (!outValue)
	{
		return false;
	}
	const auto it = m_ao.constFind(port);
	if (it != m_ao.constEnd())
	{
		*outValue = it.value();
		return true;
	}
	*outValue = 0.0;
	return true;
}

bool NamedSignalIoSink::getAnalogInput(const int port, double* outValue) const
{
	if (!outValue)
	{
		return false;
	}
	const auto it = m_ai.constFind(port);
	if (it != m_ai.constEnd())
	{
		*outValue = it.value();
		return true;
	}
	*outValue = 0.0;
	return true;
}

void NamedSignalIoSink::setDigitalInput(const int port, const bool value)
{
	m_di[port] = value;
	if (m_diForced.contains(port))
	{
		m_diForced[port] = value;
	}
	emit ioValuesChanged();
}

void NamedSignalIoSink::setDigitalInputForced(const int port, const bool value)
{
	m_diForced[port] = value;
	m_di[port] = value;
	emit ioValuesChanged();
}

void NamedSignalIoSink::clearDigitalInputForced(const int port)
{
	m_diForced.remove(port);
	emit ioValuesChanged();
}

bool NamedSignalIoSink::isDigitalInputForced(const int port) const
{
	return m_diForced.contains(port);
}

void NamedSignalIoSink::setAnalogInput(const int port, const double value)
{
	m_ai[port] = value;
	emit ioValuesChanged();
}
