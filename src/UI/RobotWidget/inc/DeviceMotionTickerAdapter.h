#ifndef ROBOTWIDGET_DEVICEMOTIONTICKERADAPTER_H
#define ROBOTWIDGET_DEVICEMOTIONTICKERADAPTER_H

/// @file DeviceMotionTickerAdapter.h
/// @brief DevicePoseMotionPlayer → IDeviceMotionTicker

#include "robotwidget_global.h"

#include "IDeviceMotionTicker.h"

#include <QPointer>

class DevicePoseMotionPlayer;
class IRobotMainWindowHost;

class ROBOTWIDGET_EXPORT DeviceMotionTickerAdapter : public IDeviceMotionTicker
{
public:
	void setPlayer(DevicePoseMotionPlayer* player) { m_player = player; }
	void setHost(IRobotMainWindowHost* host) { m_host = host; }

	bool isBusy() const override;
	bool startPose(const QString& deviceId, const QString& poseName, const std::vector<double>& targetQ,
				   double durationSec) override;
	bool startNamedPose(const QString& deviceId, const QString& poseId, double durationSec) override;
	void tick(double dtSec) override;
	void stopDevice(const QString& deviceId) override;
	void stopAll() override;
	bool isDeviceBusy(const QString& deviceId) const override;

private:
	DevicePoseMotionPlayer* m_player = nullptr;
	IRobotMainWindowHost* m_host = nullptr;
};

#endif // ROBOTWIDGET_DEVICEMOTIONTICKERADAPTER_H
