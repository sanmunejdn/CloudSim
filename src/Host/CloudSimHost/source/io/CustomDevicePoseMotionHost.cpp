/// @file CustomDevicePoseMotionHost.cpp
/// @brief Web/Headless 自定义设备姿态插值

#include "CustomDevicePoseMotionHost.h"

#include "CustomDeviceBackendData.h"
#include "CustomDeviceHostOps.h"
#include "DocumentHost.h"

#include <QJsonArray>
#include <QJsonObject>
#include <QTimer>

#include <algorithm>
#include <cmath>

namespace cloudsim::host
{
namespace
{
constexpr int kTickMs = 16;

QJsonArray doublesToJsonArr(const std::vector<double>& q)
{
	QJsonArray a;
	for (double v : q)
		a.append(v);
	return a;
}
} // namespace

CustomDevicePoseMotionHost::CustomDevicePoseMotionHost(DocumentHost& host, QObject* parent)
	: QObject(parent ? parent : &host), m_host(host)
{
}

CustomDevicePoseMotionHost& CustomDevicePoseMotionHost::forHost(DocumentHost& host)
{
	CustomDevicePoseMotionHost* existing = host.findChild<CustomDevicePoseMotionHost*>(QString(), Qt::FindDirectChildrenOnly);
	if (existing)
		return *existing;
	return *new CustomDevicePoseMotionHost(host, &host);
}

void CustomDevicePoseMotionHost::ensureTimer()
{
	if (m_timer)
		return;
	m_timer = new QTimer(this);
	m_timer->setInterval(kTickMs);
	connect(m_timer, &QTimer::timeout, this, &CustomDevicePoseMotionHost::onTick);
}

bool CustomDevicePoseMotionHost::applyQNow(const QString& deviceId, const std::vector<double>& q)
{
	QJsonObject body;
	body.insert(QStringLiteral("q"), doublesToJsonArr(q));
	QString e;
	return applyCustomDeviceQ(m_host, deviceId, body, &e);
}

bool CustomDevicePoseMotionHost::start(const QString& deviceId, const std::vector<double>& targetQ,
									   const double durationSec)
{
	if (deviceId.isEmpty() || targetQ.empty())
		return false;
	const auto device =
		std::dynamic_pointer_cast<CustomDeviceBackendData>(m_host.findObject(deviceId.toStdString()));
	if (!device)
		return false;
	device->syncAxesFromJoints();
	device->ensureQSize();
	std::vector<double> q0 = device->qValues();
	std::vector<double> q1 = targetQ;
	const size_t n = device->axes().axes.size();
	if (n == 0)
		return false;
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
			m_timer->stop();
		return ok;
	}

	ActiveMotion motion;
	motion.deviceId = deviceId;
	motion.q0 = std::move(q0);
	motion.q1 = std::move(q1);
	motion.durationSec = durationSec;
	motion.elapsedSec = 0.0;
	m_active.insert(deviceId, motion);
	ensureTimer();
	if (!m_timer->isActive())
		m_timer->start();
	return true;
}

void CustomDevicePoseMotionHost::stopDevice(const QString& deviceId)
{
	if (!m_active.remove(deviceId))
		return;
	if (m_active.isEmpty() && m_timer)
		m_timer->stop();
}

void CustomDevicePoseMotionHost::stopAll()
{
	if (m_active.isEmpty())
		return;
	m_active.clear();
	if (m_timer)
		m_timer->stop();
}

void CustomDevicePoseMotionHost::onTick()
{
	if (m_active.isEmpty())
	{
		if (m_timer)
			m_timer->stop();
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
			q[i] = m.q0[i] + (m.q1[i] - m.q0[i]) * t;
		(void)applyQNow(m.deviceId, q);
		if (t >= 1.0 - 1e-9)
			finished << m.deviceId;
	}
	for (const QString& id : finished)
		m_active.remove(id);
	if (m_active.isEmpty() && m_timer)
		m_timer->stop();
}

} // namespace cloudsim::host
