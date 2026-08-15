#ifndef ROBOTWIDGET_CUSTOMDEVICESIMSERVICE_H
#define ROBOTWIDGET_CUSTOMDEVICESIMSERVICE_H

/// @file CustomDeviceSimService.h
/// @brief 设备姿态播放 / DI 沿驱动生命周期与接线

#include "robotwidget_global.h"

#include <QObject>

class DeviceCommandPageWidget;
class DevicePoseMotionPlayer;
class DevicePoseSignalDriver;
class IRobotMainWindowHost;
class IoSignalNetworkService;

class ROBOTWIDGET_EXPORT CustomDeviceSimService : public QObject
{
	Q_OBJECT

public:
	explicit CustomDeviceSimService(QObject* parent = nullptr);

	DevicePoseMotionPlayer* motionPlayer() const { return m_player; }
	DevicePoseSignalDriver* signalDriver() const { return m_driver; }

	void setHost(IRobotMainWindowHost* host);
	void wire(IRobotMainWindowHost* host, IoSignalNetworkService* network, DeviceCommandPageWidget* deviceCmdPage);

	void resetEdgeState();

private:
	DevicePoseMotionPlayer* m_player = nullptr;
	DevicePoseSignalDriver* m_driver = nullptr;
};

#endif // ROBOTWIDGET_CUSTOMDEVICESIMSERVICE_H
