#ifndef ROBOTSCENE_IROBOTIOSINK_H
#define ROBOTSCENE_IROBOTIOSINK_H

/// @file IRobotIoSink.h
/// @brief Digital/analog IO sink for robot program execution (simulation or hardware).

#include "robot_scene_global.h"

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
};

#endif // ROBOTSCENE_IROBOTIOSINK_H
