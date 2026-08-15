#ifndef ROBOTWIDGET_DEVICEPOSEMOTIONPLAYER_H
#define ROBOTWIDGET_DEVICEPOSEMOTIONPLAYER_H

/// @file DevicePoseMotionPlayer.h
/// @brief 自定义设备姿态插值播放（与机器人程序执行器隔离）

#include "robotwidget_global.h"

#include <QHash>
#include <QObject>
#include <QString>
#include <vector>

class IRobotMainWindowHost;
class QTimer;

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

signals:
	void statusChanged();
	void motionFinished(const QString& deviceId);

private slots:
	void onTick();

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
	void ensureTimer();

	IRobotMainWindowHost* m_host = nullptr;
	QTimer* m_timer = nullptr;
	QHash<QString, ActiveMotion> m_active;
};

#endif // ROBOTWIDGET_DEVICEPOSEMOTIONPLAYER_H
