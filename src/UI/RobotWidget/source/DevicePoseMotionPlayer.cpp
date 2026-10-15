/// @file DevicePoseMotionPlayer.cpp
/// @brief 设备姿态运动播放（无独立定时器）

#include "DevicePoseMotionPlayer.h"

#include "CustomDeviceBackendData.h"
#include "CustomDeviceKinematics.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"

#include <algorithm>
#include <cmath>

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
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(deviceId.toStdString()));
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

bool DevicePoseMotionPlayer::start(const QString& deviceId, const QString& poseName, const std::vector<double>& targetQ,
								   const double durationSec)
{
	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	if (!doc || deviceId.isEmpty() || targetQ.empty())
	{
		return false;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(deviceId.toStdString()));
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
	emit statusChanged();
	return true;
}

void DevicePoseMotionPlayer::stopDevice(const QString& deviceId)
{
	if (!m_active.remove(deviceId))
	{
		return;
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
	emit statusChanged();
}

void DevicePoseMotionPlayer::tick(const double dtSec)
{
	if (m_active.isEmpty())
	{
		return;
	}
	const double dt = std::max(0.0, dtSec);
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
	emit statusChanged();
}
