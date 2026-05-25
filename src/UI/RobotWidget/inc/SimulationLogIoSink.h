#pragma once

#include "IRobotIoSink.h"
#include "robotwidget_global.h"

#include <QHash>

/// 仿真数字 IO 内存态（输出变更经 RunLogger 记录）
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
