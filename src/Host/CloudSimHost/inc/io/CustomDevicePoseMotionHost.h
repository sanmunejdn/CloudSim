#ifndef CLOUDSIMHOST_CUSTOMDEVICEPOSEMOTIONHOST_H
#define CLOUDSIMHOST_CUSTOMDEVICEPOSEMOTIONHOST_H

/// @file CustomDevicePoseMotionHost.h
/// @brief Web/Headless：DI 绑定姿态插值（对齐桌面 DevicePoseMotionPlayer）

#include "cloudsim_host_global.h"

#include <QHash>
#include <QObject>
#include <QString>
#include <vector>

class QTimer;

namespace cloudsim::host
{
class DocumentHost;

/// 挂在 DocumentHost 下；durationSec<=0 瞬时 applyQ
class CLOUDSIM_HOST_EXPORT CustomDevicePoseMotionHost : public QObject
{
	Q_OBJECT
public:
	explicit CustomDevicePoseMotionHost(DocumentHost& host, QObject* parent = nullptr);

	bool start(const QString& deviceId, const std::vector<double>& targetQ, double durationSec);
	void stopDevice(const QString& deviceId);
	void stopAll();

	static CustomDevicePoseMotionHost& forHost(DocumentHost& host);

private slots:
	void onTick();

private:
	struct ActiveMotion
	{
		QString deviceId;
		std::vector<double> q0;
		std::vector<double> q1;
		double durationSec = 1.0;
		double elapsedSec = 0.0;
	};

	void ensureTimer();
	bool applyQNow(const QString& deviceId, const std::vector<double>& q);

	DocumentHost& m_host;
	QTimer* m_timer = nullptr;
	QHash<QString, ActiveMotion> m_active;
};

} // namespace cloudsim::host

#endif
