/// @file ExternalPlcBridge.cpp
/// @brief 外接 PLC 镜像桥（PlcComm 读写在工作线程）

#include "ExternalPlcBridge.h"

#include "IPlcCommClient.h"
#include "IoSignalNetworkService.h"
#include "NamedSignalIoSink.h"
#include "NamedSignalTable.h"
#include "PlcCommTypes.h"

#include <QMetaObject>
#include <QPointer>
#include <utility>

namespace external_plc_bridge
{
// 无 Q_OBJECT：用 Queued lambda 回桥，避免 cpp 内 moc
class Worker final : public QObject
{
public:
	explicit Worker(QObject* parent = nullptr) : QObject(parent) {}

	bool connectLink(const ExternalPlcLinkConfig& link, QString* err)
	{
		if (m_client)
		{
			m_client->disconnect();
			m_client.reset();
		}
		m_handles.clear();
		m_client = createPlcCommClient();
		PlcConnectionConfig cfg;
		if (link.protocol.compare(QStringLiteral("abEip"), Qt::CaseInsensitive) == 0)
		{
			cfg.protocol = PlcProtocol::AbEip;
			cfg.port = link.port > 0 ? static_cast<uint16_t>(link.port) : static_cast<uint16_t>(44818);
			cfg.cpu = link.cpu.toStdString();
		}
		else
		{
			cfg.protocol = PlcProtocol::ModbusTcp;
			cfg.port = link.port > 0 ? static_cast<uint16_t>(link.port) : static_cast<uint16_t>(502);
		}
		cfg.gateway = link.gateway.toStdString();
		cfg.path = link.path.toStdString();
		cfg.timeoutMs = link.timeoutMs > 0 ? link.timeoutMs : 3000;
		if (!m_client->connect(cfg))
		{
			if (err)
			{
				*err = QString::fromStdString(m_client->lastError());
			}
			m_client.reset();
			return false;
		}
		return true;
	}

	void disconnectLink()
	{
		if (m_client)
		{
			m_client->disconnect();
			m_client.reset();
		}
		m_handles.clear();
	}

	bool syncOnce(const QVector<PlcSignalMapEntry>& map, const QVector<QPair<QString, bool>>& outs,
				  QHash<QString, bool>* inValues, QString* err)
	{
		if (!m_client || !m_client->isConnected())
		{
			if (err)
			{
				*err = QStringLiteral("PLC not connected");
			}
			return false;
		}
		for (const auto& out : outs)
		{
			const int h = ensureHandle(out.first);
			if (h < 0)
			{
				if (err)
				{
					*err = QString::fromStdString(m_client->lastError());
				}
				return false;
			}
			PlcTagValue v;
			v.data = {static_cast<uint8_t>(out.second ? 1 : 0)};
			if (!m_client->writeTag(h, v))
			{
				if (err)
				{
					*err = QString::fromStdString(m_client->lastError());
				}
				return false;
			}
		}
		QHash<QString, bool> localIn;
		for (const PlcSignalMapEntry& e : map)
		{
			if (e.direction != PlcMapDirection::In)
			{
				continue;
			}
			const int h = ensureHandle(e.plcTag);
			if (h < 0)
			{
				if (err)
				{
					*err = QString::fromStdString(m_client->lastError());
				}
				return false;
			}
			PlcTagValue v;
			if (!m_client->readTag(h, v) || v.data.empty())
			{
				if (err)
				{
					*err = QString::fromStdString(m_client->lastError());
				}
				return false;
			}
			localIn.insert(e.ownerId + QLatin1Char('|') + e.signal, v.data.front() != 0);
		}
		if (inValues)
		{
			*inValues = localIn;
		}
		return true;
	}

private:
	int ensureHandle(const QString& plcTag)
	{
		const auto it = m_handles.constFind(plcTag);
		if (it != m_handles.constEnd())
		{
			return it.value();
		}
		if (!m_client)
		{
			return -1;
		}
		PlcTagSpec spec;
		spec.name = plcTag.toStdString();
		spec.elemCount = 1;
		spec.elemSize = 1;
		const int h = m_client->addTag(spec);
		if (h >= 0)
		{
			m_handles.insert(plcTag, h);
		}
		return h;
	}

	std::unique_ptr<IPlcCommClient> m_client;
	QHash<QString, int> m_handles;
};
} // namespace external_plc_bridge

ExternalPlcBridge::ExternalPlcBridge(QObject* parent) : QObject(parent) {}

ExternalPlcBridge::~ExternalPlcBridge()
{
	stopWorker();
}

void ExternalPlcBridge::setNetwork(IoSignalNetworkService* network)
{
	m_network = network;
}

void ExternalPlcBridge::setLinkConfig(const ExternalPlcLinkConfig& config)
{
	m_link = config;
}

void ExternalPlcBridge::setSignalMap(const QVector<PlcSignalMapEntry>& map)
{
	m_map = map;
}

void ExternalPlcBridge::ensureWorker()
{
	if (m_thread)
	{
		return;
	}
	m_thread = new QThread(this);
	auto* worker = new external_plc_bridge::Worker();
	m_worker = worker;
	worker->moveToThread(m_thread);
	connect(m_thread, &QThread::finished, worker, &QObject::deleteLater);
	m_thread->start();
}

void ExternalPlcBridge::stopWorker()
{
	if (!m_thread)
	{
		return;
	}
	if (auto* worker = static_cast<external_plc_bridge::Worker*>(m_worker))
	{
		QMetaObject::invokeMethod(
			worker,
			[worker]()
			{
				worker->disconnectLink();
			},
			Qt::BlockingQueuedConnection);
	}
	m_thread->quit();
	m_thread->wait(3000);
	m_thread = nullptr;
	m_worker = nullptr;
	m_connected = false;
	m_syncBusy = false;
}

bool ExternalPlcBridge::connectPlc(QString* err)
{
	ensureWorker();
	auto* worker = static_cast<external_plc_bridge::Worker*>(m_worker);
	if (!worker)
	{
		if (err)
		{
			*err = QStringLiteral("PLC worker unavailable");
		}
		return false;
	}
	m_disconnectWarned = false;
	bool ok = false;
	QString localErr;
	const ExternalPlcLinkConfig link = m_link;
	QMetaObject::invokeMethod(
		worker,
		[worker, link, &ok, &localErr]()
		{
			ok = worker->connectLink(link, &localErr);
		},
		Qt::BlockingQueuedConnection);
	m_connected = ok;
	if (!ok && err)
	{
		*err = localErr;
	}
	emit plcConnectionChanged(ok);
	return ok;
}

void ExternalPlcBridge::disconnectPlc()
{
	if (auto* worker = static_cast<external_plc_bridge::Worker*>(m_worker))
	{
		QMetaObject::invokeMethod(
			worker,
			[worker]()
			{
				worker->disconnectLink();
			},
			Qt::BlockingQueuedConnection);
	}
	m_connected = false;
	m_cachedIn.clear();
	m_pendingOut.clear();
	m_syncBusy = false;
	emit plcConnectionChanged(false);
}

QString ExternalPlcBridge::mapKey(const QString& ownerId, const QString& signal) const
{
	return ownerId + QLatin1Char('|') + signal;
}

void ExternalPlcBridge::onAfterIoPhase()
{
	if (!m_network || !m_connected)
	{
		return;
	}

	for (const PlcSignalMapEntry& e : m_map)
	{
		if (e.direction != PlcMapDirection::In)
		{
			continue;
		}
		const QString key = mapKey(e.ownerId, e.signal);
		if (!m_cachedIn.contains(key))
		{
			continue;
		}
		NamedSignalIoSink* sink = m_network->sink(e.ownerId);
		RobotIo::NamedSignalTable* table = m_network->table(e.ownerId);
		if (!sink || !table)
		{
			continue;
		}
		const RobotIo::SignalDef* def = table->findByName(e.signal.toStdString());
		if (!def || def->kind != RobotIo::SignalKind::DI)
		{
			continue;
		}
		sink->setDigitalInputForced(def->port, m_cachedIn.value(key));
	}

	QVector<QPair<QString, bool>> outs;
	for (const PlcSignalMapEntry& e : m_map)
	{
		if (e.direction != PlcMapDirection::Out)
		{
			continue;
		}
		NamedSignalIoSink* sink = m_network->sink(e.ownerId);
		RobotIo::NamedSignalTable* table = m_network->table(e.ownerId);
		if (!sink || !table)
		{
			continue;
		}
		const RobotIo::SignalDef* def = table->findByName(e.signal.toStdString());
		if (!def || def->kind != RobotIo::SignalKind::DO)
		{
			continue;
		}
		bool v = false;
		if (!sink->getDigitalOutput(def->port, &v))
		{
			continue;
		}
		outs.push_back(qMakePair(e.plcTag, v));
	}

	if (m_syncBusy || !m_worker)
	{
		return;
	}
	auto* worker = static_cast<external_plc_bridge::Worker*>(m_worker);
	if (!worker)
	{
		return;
	}
	m_syncBusy = true;
	const QVector<PlcSignalMapEntry> map = m_map;
	const QPointer<ExternalPlcBridge> self(this);
	QMetaObject::invokeMethod(
		worker,
		[worker, map, outs, self]()
		{
			QHash<QString, bool> inValues;
			QString err;
			const bool ok = worker->syncOnce(map, outs, &inValues, &err);
			if (!self)
			{
				return;
			}
			QMetaObject::invokeMethod(
				self.data(),
				[self, ok, err, inValues]()
				{
					if (self)
					{
						self->onWorkerFinished(ok, err, inValues);
					}
				},
				Qt::QueuedConnection);
		},
		Qt::QueuedConnection);
}

void ExternalPlcBridge::onWorkerFinished(bool ok, const QString& error, const QHash<QString, bool>& inValues)
{
	m_syncBusy = false;
	if (!ok)
	{
		if (!m_disconnectWarned)
		{
			m_disconnectWarned = true;
			emit plcWarning(error.isEmpty() ? QStringLiteral("PLC sync failed; simulation continues") : error);
		}
		if (m_connected)
		{
			m_connected = false;
			emit plcConnectionChanged(false);
		}
		return;
	}
	m_disconnectWarned = false;
	m_cachedIn = inValues;
}

void ExternalPlcBridge::onWorkerDisconnected()
{
	m_connected = false;
	m_syncBusy = false;
	emit plcConnectionChanged(false);
}
