#ifndef ROBOTWIDGET_DEVICEPOSEMOTIONPLAYER_H
#define ROBOTWIDGET_DEVICEPOSEMOTIONPLAYER_H

/// @file DevicePoseMotionPlayer.h
/// @brief 自定义设备姿态插值（由 SimulationKernel / Controller 同拍 tick，无独立 QTimer）

#include "robotwidget_global.h"

#include <QHash>
#include <QObject>
#include <QString>
#include <vector>

class IRobotMainWindowHost;

class ROBOTWIDGET_EXPORT DevicePoseMotionPlayer : public QObject
{
	Q_OBJECT

public:
	explicit DevicePoseMotionPlayer(QObject* parent = nullptr);

	void setHost(IRobotMainWindowHost* host);

	bool isBusy() const { return !m_active.isEmpty(); }
	bool isDeviceBusy(const QString& deviceId) const;
	QString statusText() const;

	/// durationSec<=0：瞬移；同设备再次 start 打断并改目标
	bool start(const QString& deviceId, const QString& poseName, const std::vector<double>& targetQ,
			   double durationSec);
	void stopDevice(const QString& deviceId);
	void stopAll();

	/// 由统一仿真时钟驱动
	void tick(double dtSec);

signals:
	void statusChanged();
	void motionFinished(const QString& deviceId);

private:
	struct ActiveMotion
	{
		QString deviceId;
		QString poseName;
		std::vector<double> q0;
		std::vector<double> q1;
		double durationSec = 0.0;
		double elapsedSec = 0.0;
	};

	bool applyQNow(const QString& deviceId, const std::vector<double>& q);

	IRobotMainWindowHost* m_host = nullptr;
	QHash<QString, ActiveMotion> m_active;
};

#endif // ROBOTWIDGET_DEVICEPOSEMOTIONPLAYER_H
