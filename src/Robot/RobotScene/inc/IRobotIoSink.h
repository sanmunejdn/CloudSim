#ifndef ROBOTSCENE_IROBOTIOSINK_H
#define ROBOTSCENE_IROBOTIOSINK_H

/// @file IRobotIoSink.h
/// @brief Digital/analog IO sink for robot program execution (simulation or hardware).

#include "robot_scene_global.h"

#include <string>

/// 后续真机可换 Backend；本期仅 Simulation 验收
enum class ROBOT_SCENE_API RobotIoSinkBackend
{
	Simulation = 0,
	PlcStub
};

/// Digital/analog IO sink for robot program execution (simulation or hardware).
class ROBOT_SCENE_API IRobotIoSink
{
public:
	virtual ~IRobotIoSink() = default;

	virtual bool getDigitalInput(int port, bool* outValue) const
	{
		(void)port;
		if (outValue)
		{
			*outValue = false;
		}
		return false;
	}

	virtual void setDigitalOutput(int port, bool value) = 0;
	virtual void setAnalogOutput(int port, double value) = 0;

	/// 程序绑定 signalName 时解析端口；默认忽略名称
	virtual int resolveNamedPort(const std::string& signalName, int fallbackPort) const
	{
		(void)signalName;
		return fallbackPort;
	}
};

#endif // ROBOTSCENE_IROBOTIOSINK_H

