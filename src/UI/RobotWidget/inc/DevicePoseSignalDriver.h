#ifndef ROBOTWIDGET_DEVICEPOSESIGNALDRIVER_H
#define ROBOTWIDGET_DEVICEPOSESIGNALDRIVER_H

/// @file DevicePoseSignalDriver.h
/// @brief 设备本地 DI 上升沿 → 命名姿态

#include "robotwidget_global.h"

#include <QHash>
#include <QObject>
#include <QString>

class DevicePoseMotionPlayer;
class IRobotMainWindowHost;
class IoSignalNetworkService;

class ROBOTWIDGET_EXPORT DevicePoseSignalDriver : public QObject
{
	Q_OBJECT

public:
	explicit DevicePoseSignalDriver(QObject* parent = nullptr);

	void setHost(IRobotMainWindowHost* host);
	void setNetwork(IoSignalNetworkService* network);
	void setMotionPlayer(DevicePoseMotionPlayer* player);

	void resetEdgeState();

private slots:
	void onOwnerIoChanged(const QString& ownerId);

private:
	bool readDeviceDi(const QString& deviceId, const QString& signalName, bool* outValue) const;
	void processDeviceRisingEdges(const QString& deviceId);

	IRobotMainWindowHost* m_host = nullptr;
	IoSignalNetworkService* m_network = nullptr;
	DevicePoseMotionPlayer* m_player = nullptr;
	/// key = deviceId|signalName
	QHash<QString, bool> m_lastDi;
};

#endif // ROBOTWIDGET_DEVICEPOSESIGNALDRIVER_H
