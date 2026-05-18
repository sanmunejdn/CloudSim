#include "SimulationLogIoSink.h"

#include "RunLogger.h"

void SimulationLogIoSink::setDigitalOutput(const int port, const bool value)
{
	m_digitalOut[port] = value;
	RunLogger::info(std::string("Simulation IO: DO ") + std::to_string(port) + " = " + (value ? "1" : "0"));
}

void SimulationLogIoSink::setAnalogOutput(const int port, const double value)
{
	m_analogOut[port] = value;
	RunLogger::info(std::string("Simulation IO: AO ") + std::to_string(port) + " = " + std::to_string(value));
}

bool SimulationLogIoSink::getDigitalInput(const int port, bool* outValue) const
{
	if (!outValue)
	{
		return false;
	}
	const auto it = m_digitalOut.constFind(port);
	if (it != m_digitalOut.constEnd())
	{
		*outValue = it.value();
		return true;
	}
	*outValue = false;
	return true;
}
