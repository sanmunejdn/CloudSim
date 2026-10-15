/// @file DeviceMotionTickerAdapter.cpp
/// @brief 设备运动 ticker 适配

#include "DeviceMotionTickerAdapter.h"

#include "CustomDeviceBackendData.h"
#include "DevicePoseMotionPlayer.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"

bool DeviceMotionTickerAdapter::isBusy() const
{
	return m_player && m_player->isBusy();
}

bool DeviceMotionTickerAdapter::startPose(const QString& deviceId, const QString& poseName,
										  const std::vector<double>& targetQ, double durationSec)
{
	return m_player && m_player->start(deviceId, poseName, targetQ, durationSec);
}

bool DeviceMotionTickerAdapter::startNamedPose(const QString& deviceId, const QString& poseId, double durationSec)
{
	if (!m_player || !m_host || deviceId.isEmpty() || poseId.isEmpty())
	{
		return false;
	}
	IRobotDocumentHost* doc = m_host->document();
	if (!doc)
	{
		return false;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(deviceId.toStdString()));
	if (!device)
	{
		return false;
	}
	const CustomDeviceNamedPose* pose = device->findNamedPose(poseId.toStdString());
	if (!pose)
	{
		return false;
	}
	return m_player->start(deviceId, QString::fromStdString(pose->name), pose->q, durationSec);
}

void DeviceMotionTickerAdapter::tick(double dtSec)
{
	if (m_player)
	{
		m_player->tick(dtSec);
	}
}

void DeviceMotionTickerAdapter::stopDevice(const QString& deviceId)
{
	if (m_player)
	{
		m_player->stopDevice(deviceId);
	}
}

void DeviceMotionTickerAdapter::stopAll()
{
	if (m_player)
	{
		m_player->stopAll();
	}
}

bool DeviceMotionTickerAdapter::isDeviceBusy(const QString& deviceId) const
{
	return m_player && m_player->isDeviceBusy(deviceId);
}
