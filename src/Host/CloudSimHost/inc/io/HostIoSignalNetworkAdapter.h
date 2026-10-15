#ifndef CLOUDSIMHOST_HOSTIOSIGNALNETWORKADAPTER_H
#define CLOUDSIMHOST_HOSTIOSIGNALNETWORKADAPTER_H

/// @file HostIoSignalNetworkAdapter.h
/// @brief Host IoSignalNetwork → RobotScene IIoSignalNetwork（下一拍采样由 Kernel IoPhase 负责）

#include "cloudsim_host_global.h"

#include "IIoSignalNetwork.h"
#include "IoSignalNetwork.h"

namespace cloudsim::host
{

class CLOUDSIM_HOST_EXPORT HostIoSignalNetworkAdapter : public IIoSignalNetwork
{
public:
	explicit HostIoSignalNetworkAdapter(IoSignalNetwork* network = nullptr) : m_network(network) {}

	void setNetwork(IoSignalNetwork* network) { m_network = network; }

	QStringList ownerIds() const override { return m_network ? m_network->ownerIds() : QStringList(); }

	IRobotIoSink* sink(const QString& ownerId) override
	{
		(void)ownerId;
		// Host 网本身即 IRobotIoSink（primary）；多 Owner 细粒度 sink 后续扩展
		return m_network;
	}

	void propagateFrom(const QString& ownerId) override
	{
		if (m_network)
		{
			m_network->propagateFrom(ownerId);
		}
	}

	void propagateAll() override
	{
		if (m_network)
		{
			m_network->propagateAll();
		}
	}

private:
	IoSignalNetwork* m_network = nullptr;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_HOSTIOSIGNALNETWORKADAPTER_H
