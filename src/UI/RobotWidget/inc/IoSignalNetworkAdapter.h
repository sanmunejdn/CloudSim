#ifndef ROBOTWIDGET_IOSIGNALNETWORKADAPTER_H
#define ROBOTWIDGET_IOSIGNALNETWORKADAPTER_H

/// @file IoSignalNetworkAdapter.h
/// @brief IoSignalNetworkService → IIoSignalNetwork

#include "robotwidget_global.h"

#include "IIoSignalNetwork.h"
#include "IoSignalNetworkService.h"

class ROBOTWIDGET_EXPORT IoSignalNetworkAdapter : public IIoSignalNetwork
{
public:
	explicit IoSignalNetworkAdapter(IoSignalNetworkService* service = nullptr) : m_service(service) {}

	void setService(IoSignalNetworkService* service) { m_service = service; }

	QStringList ownerIds() const override { return m_service ? m_service->ownerIds() : QStringList(); }

	IRobotIoSink* sink(const QString& ownerId) override
	{
		return m_service ? static_cast<IRobotIoSink*>(m_service->sink(ownerId)) : nullptr;
	}

	void propagateFrom(const QString& ownerId) override
	{
		if (m_service)
		{
			m_service->propagateFrom(ownerId);
		}
	}

	void propagateAll() override
	{
		if (m_service)
		{
			m_service->propagateAll();
		}
	}

private:
	IoSignalNetworkService* m_service = nullptr;
};

#endif // ROBOTWIDGET_IOSIGNALNETWORKADAPTER_H
