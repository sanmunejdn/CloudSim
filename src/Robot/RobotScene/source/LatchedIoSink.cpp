/// @file LatchedIoSink.cpp
/// @brief 下一拍 DI 锁存

#include "LatchedIoSink.h"

LatchedIoSink::LatchedIoSink(IRobotIoSink* backend) : m_backend(backend) {}

void LatchedIoSink::capturePortsToNext(const QList<int>& digitalPorts)
{
	if (!m_backend)
	{
		return;
	}
	for (int port : digitalPorts)
	{
		bool v = false;
		if (m_backend->getDigitalInput(port, &v))
		{
			m_nextDi.insert(port, v);
		}
	}
}

void LatchedIoSink::swapNextToCurrent()
{
	m_currentDi = m_nextDi;
}

void LatchedIoSink::clearLatches()
{
	m_currentDi.clear();
	m_nextDi.clear();
}

bool LatchedIoSink::getDigitalInput(int port, bool* outValue) const
{
	if (m_currentDi.contains(port))
	{
		if (outValue)
		{
			*outValue = m_currentDi.value(port);
		}
		return true;
	}
	// 首拍尚未锁存时回退底层，避免 Wait 永久读 false
	if (m_backend)
	{
		return m_backend->getDigitalInput(port, outValue);
	}
	if (outValue)
	{
		*outValue = false;
	}
	return false;
}

void LatchedIoSink::setDigitalOutput(int port, bool value)
{
	if (m_backend)
	{
		m_backend->setDigitalOutput(port, value);
	}
}

void LatchedIoSink::setAnalogOutput(int port, double value)
{
	if (m_backend)
	{
		m_backend->setAnalogOutput(port, value);
	}
}

int LatchedIoSink::resolveNamedPort(const std::string& signalName, int fallbackPort) const
{
	if (m_backend)
	{
		return m_backend->resolveNamedPort(signalName, fallbackPort);
	}
	return fallbackPort;
}
