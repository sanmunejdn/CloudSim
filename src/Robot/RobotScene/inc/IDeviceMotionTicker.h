#ifndef ROBOTSCENE_IDEVICEMOTIONTICKER_H
#define ROBOTSCENE_IDEVICEMOTIONTICKER_H

/// @file IDeviceMotionTicker.h
/// @brief 设备姿态插值由 Kernel 同拍驱动（实现可在 RobotWidget）

#include "robot_scene_global.h"

#include <QString>
#include <vector>

class ROBOT_SCENE_API IDeviceMotionTicker
{
public:
	virtual ~IDeviceMotionTicker() = default;
	virtual bool isBusy() const = 0;
	virtual bool startPose(const QString& deviceId, const QString& poseName, const std::vector<double>& targetQ,
						   double durationSec) = 0;
	/// 按命名姿态 id 解析 q 后启动；设备程序 GoNamedPose 走此入口
	virtual bool startNamedPose(const QString& deviceId, const QString& poseId, double durationSec)
	{
		(void)deviceId;
		(void)poseId;
		(void)durationSec;
		return false;
	}
	virtual void tick(double dtSec) = 0;
	virtual void stopDevice(const QString& deviceId) = 0;
	virtual void stopAll() = 0;
	virtual bool isDeviceBusy(const QString& deviceId) const = 0;
};

#endif // ROBOTSCENE_IDEVICEMOTIONTICKER_H
