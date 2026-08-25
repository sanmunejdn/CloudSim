/// @file DevicePoseMotionPlayer.cpp
/// @brief 设备姿态运动播放

#include "DevicePoseMotionPlayer.h"

#include "CustomDeviceBackendData.h"
#include "CustomDeviceKinematics.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"

#include <QTimer>
#include <algorithm>
#include <cmath>

namespace
{
constexpr int kTickMs = 16;
}

DevicePoseMotionPlayer::DevicePoseMotionPlayer(QObject* parent) : QObject(parent) {}

void DevicePoseMotionPlayer::setHost(IRobotMainWindowHost* host)
{
	m_host = host;
}

bool DevicePoseMotionPlayer::isDeviceBusy(const QString& deviceId) const
{
	return m_active.contains(deviceId);
}

QString DevicePoseMotionPlayer::statusText() const
{
	if (m_active.isEmpty())
	{
		return QString();
	}
	const ActiveMotion& m = *m_active.constBegin();
	if (m_active.size() == 1)
	{
		return m.poseName.isEmpty() ? m.deviceId : m.poseName;
	}
	return QStringLiteral("%1 (+%2)").arg(m.poseName.isEmpty() ? m.deviceId : m.poseName).arg(m_active.size() - 1);
}

bool DevicePoseMotionPlayer::applyQNow(const QString& deviceId, const std::vector<double>& q)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || deviceId.isEmpty())
	{
		return false;
	}
	const auto device =
		std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(deviceId.toStdString()));
	if (!device)
	{
		return false;
	}
	if (!CustomDeviceKinematics::applyQ(*device, &doc->backend(), doc->poseSink(), &q))
	{
		return false;
	}
	if (m_host)
	{
		m_host->flushCustomDeviceLinkGeometryVisual(deviceId);
	}
	if (m_host && m_host->osgView())
	{
		m_host->osgView()->requestRedraw();
	}
	return true;
}

void DevicePoseMotionPlayer::ensureTimer()
{
	if (!m_timer)
	{
		m_timer = new QTimer(this);
		m_timer->setInterval(kTickMs);
		connect(m_timer, &QTimer::timeout, this, &DevicePoseMotionPlayer::onTick);
	}
}

bool DevicePoseMotionPlayer::start(const QString& deviceId, const QString& poseName,
								   const std::vector<double>& targetQ, const double durationSec)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || deviceId.isEmpty() || targetQ.empty())
	{
		return false;
	}
	const auto device =
		std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(deviceId.toStdString()));
	if (!device)
	{
		return false;
	}
	device->ensureQSize();
	std::vector<double> q0 = device->qValues();
	std::vector<double> q1 = targetQ;
	const size_t n = device->axes().axes.size();
	if (n == 0)
	{
		return false;
	}
	q0.resize(n, 0.0);
	q1.resize(n, 0.0);
	for (size_t i = 0; i < n; ++i)
	{
		const CustomDeviceAxisConfig& ax = device->axes().axes[i];
		q0[i] = std::clamp(q0[i], ax.lower, ax.upper);
		q1[i] = std::clamp(q1[i], ax.lower, ax.upper);
	}

	if (durationSec <= 1e-6)
	{
		m_active.remove(deviceId);
		const bool ok = applyQNow(deviceId, q1);
		if (m_active.isEmpty() && m_timer)
		{
			m_timer->stop();
		}
		emit statusChanged();
		if (ok)
		{
			emit motionFinished(deviceId);
		}
		return ok;
	}

	ActiveMotion motion;
	motion.deviceId = deviceId;
	motion.poseName = poseName;
	motion.q0 = std::move(q0);
	motion.q1 = std::move(q1);
	motion.durationSec = durationSec;
	motion.elapsedSec = 0.0;
	m_active.insert(deviceId, motion);
	ensureTimer();
	if (!m_timer->isActive())
	{
		m_timer->start();
	}
	emit statusChanged();
	return true;
}

void DevicePoseMotionPlayer::stopDevice(const QString& deviceId)
{
	if (!m_active.remove(deviceId))
	{
		return;
	}
	if (m_active.isEmpty() && m_timer)
	{
		m_timer->stop();
	}
	emit statusChanged();
}

void DevicePoseMotionPlayer::stopAll()
{
	if (m_active.isEmpty())
	{
		return;
	}
	m_active.clear();
	if (m_timer)
	{
		m_timer->stop();
	}
	emit statusChanged();
}

void DevicePoseMotionPlayer::onTick()
{
	if (m_active.isEmpty())
	{
		if (m_timer)
		{
			m_timer->stop();
		}
		return;
	}
	const double dt = static_cast<double>(kTickMs) / 1000.0;
	QStringList finished;
	for (auto it = m_active.begin(); it != m_active.end(); ++it)
	{
		ActiveMotion& m = it.value();
		m.elapsedSec += dt;
		const double t = std::min(1.0, m.elapsedSec / m.durationSec);
		std::vector<double> q(m.q0.size(), 0.0);
		for (size_t i = 0; i < q.size(); ++i)
		{
			q[i] = m.q0[i] + (m.q1[i] - m.q0[i]) * t;
		}
		(void)applyQNow(m.deviceId, q);
		if (t >= 1.0 - 1e-9)
		{
			finished << m.deviceId;
		}
	}
	for (const QString& id : finished)
	{
		m_active.remove(id);
		emit motionFinished(id);
	}
	if (m_active.isEmpty() && m_timer)
	{
		m_timer->stop();
	}
	emit statusChanged();
}
