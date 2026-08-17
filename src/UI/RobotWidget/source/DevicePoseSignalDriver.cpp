/// @file DevicePoseSignalDriver.cpp
/// @brief 设备姿态信号驱动

#include "DevicePoseSignalDriver.h"

#include "CustomDeviceBackendData.h"
#include "DevicePoseMotionPlayer.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"
#include "IoSignalNetworkService.h"
#include "NamedSignalIoSink.h"
#include "NamedSignalTable.h"

DevicePoseSignalDriver::DevicePoseSignalDriver(QObject* parent) : QObject(parent) {}

void DevicePoseSignalDriver::setHost(IRobotMainWindowHost* host)
{
	m_host = host;
}

void DevicePoseSignalDriver::setNetwork(IoSignalNetworkService* network)
{
	if (m_network)
	{
		disconnect(m_network, nullptr, this, nullptr);
	}
	m_network = network;
	if (m_network)
	{
		connect(m_network, &IoSignalNetworkService::ownerIoChanged, this, &DevicePoseSignalDriver::onOwnerIoChanged);
	}
}

void DevicePoseSignalDriver::setMotionPlayer(DevicePoseMotionPlayer* player)
{
	m_player = player;
}

void DevicePoseSignalDriver::resetEdgeState()
{
	m_lastDi.clear();
	if (m_player)
	{
		m_player->stopAll();
	}
}

bool DevicePoseSignalDriver::readDeviceDi(const QString& deviceId, const QString& signalName, bool* outValue) const
{
	if (!m_network || !outValue || deviceId.isEmpty() || signalName.isEmpty())
	{
		return false;
	}
	RobotIo::NamedSignalTable* table = m_network->table(deviceId);
	NamedSignalIoSink* sink = m_network->sink(deviceId);
	if (!table || !sink)
	{
		return false;
	}
	const RobotIo::SignalDef* def = table->findByName(signalName.toStdString());
	if (!def || def->kind != RobotIo::SignalKind::DI)
	{
		return false;
	}
	const int port = sink->resolveNamedPort(signalName.toStdString(), def->port);
	return sink->getDigitalInput(port, outValue);
}

void DevicePoseSignalDriver::processDeviceRisingEdges(const QString& deviceId)
{
	if (!m_host || !m_player || !m_network || deviceId.isEmpty())
	{
		return;
	}
	IRobotDocumentHost* doc = m_host->document();
	if (!doc)
	{
		return;
	}
	const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(deviceId.toStdString()));
	RobotIo::NamedSignalTable* table = m_network->table(deviceId);
	if (!device || !table)
	{
		return;
	}
	for (const RobotIo::SignalDef& s : table->entries())
	{
		if (s.kind != RobotIo::SignalKind::DI || s.name.empty())
		{
			continue;
		}
		const QString name = QString::fromStdString(s.name);
		const QString key = deviceId + QLatin1Char('|') + name;
		bool now = false;
		if (!readDeviceDi(deviceId, name, &now))
		{
			continue;
		}
		const bool had = m_lastDi.contains(key);
		const bool prev = had ? m_lastDi.value(key) : now;
		m_lastDi.insert(key, now);
		if (!had || !(!prev && now))
		{
			continue;
		}
		for (const CustomDevicePoseSignalBinding& b : device->poseSignalBindings())
		{
			if (!b.enabled || b.signalName != s.name)
			{
				continue;
			}
			const CustomDeviceNamedPose* pose = device->findNamedPose(b.poseId);
			if (!pose)
			{
				break;
			}
			(void)m_player->start(deviceId, QString::fromStdString(pose->name), pose->q, b.durationSec);
			break;
		}
	}
}

void DevicePoseSignalDriver::onOwnerIoChanged(const QString& ownerId)
{
	if (!m_network || m_network->ownerKind(ownerId) != IoSignalOwnerKind::Device)
	{
		return;
	}
	processDeviceRisingEdges(ownerId);
}
