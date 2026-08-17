/// @file CustomDeviceSimService.cpp
/// @brief 自定义设备仿真服务

#include "CustomDeviceSimService.h"

#include "DeviceCommandPageWidget.h"
#include "DevicePoseMotionPlayer.h"
#include "DevicePoseSignalDriver.h"
#include "IRobotMainWindowHost.h"
#include "IoSignalNetworkService.h"

CustomDeviceSimService::CustomDeviceSimService(QObject* parent) : QObject(parent)
{
	m_player = new DevicePoseMotionPlayer(this);
	m_driver = new DevicePoseSignalDriver(this);
}

void CustomDeviceSimService::setHost(IRobotMainWindowHost* host)
{
	if (m_player)
	{
		m_player->setHost(host);
	}
	if (m_driver)
	{
		m_driver->setHost(host);
	}
}

void CustomDeviceSimService::wire(IRobotMainWindowHost* host, IoSignalNetworkService* network,
								  DeviceCommandPageWidget* deviceCmdPage)
{
	setHost(host);
	if (m_driver)
	{
		m_driver->setNetwork(network);
		m_driver->setMotionPlayer(m_player);
	}
	if (deviceCmdPage)
	{
		deviceCmdPage->setHost(host);
		deviceCmdPage->setNetwork(network);
		deviceCmdPage->setMotionPlayer(m_player);
		if (host)
		{
			deviceCmdPage->setUseChinese(host->useChinese());
		}
	}
}

void CustomDeviceSimService::resetEdgeState()
{
	if (m_driver)
	{
		m_driver->resetEdgeState();
	}
}
