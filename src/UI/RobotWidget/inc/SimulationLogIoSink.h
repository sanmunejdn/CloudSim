#pragma once

#include "IRobotIoSink.h"
#include "robotwidget_global.h"

#include <QHash>

/// In-memory digital IO state for simulation (logged via RunLogger when outputs change).
class ROBOTWIDGET_EXPORT SimulationLogIoSink : public IRobotIoSink
{
public:
	void setDigitalOutput(int port, bool value) override;
	void setAnalogOutput(int port, double value) override;
	bool getDigitalInput(int port, bool* outValue) const override;

private:
	QHash<int, bool> m_digitalOut;
	QHash<int, double> m_analogOut;
};
