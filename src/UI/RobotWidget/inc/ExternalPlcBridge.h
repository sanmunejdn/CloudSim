#ifndef ROBOTWIDGET_EXTERNALPLCBRIDGE_H
#define ROBOTWIDGET_EXTERNALPLCBRIDGE_H

/// @file ExternalPlcBridge.h
/// @brief EventBased+ExternalPlc：PlcComm 与 IoSignal 双向 bool 镜像

#include "robotwidget_global.h"

#include "PlcSignalMapTypes.h"

#include <QHash>
#include <QObject>
#include <QString>
#include <QThread>
#include <QVector>
#include <memory>

class IoSignalNetworkService;
class IPlcCommClient;

class ROBOTWIDGET_EXPORT ExternalPlcBridge : public QObject
{
	Q_OBJECT

public:
	explicit ExternalPlcBridge(QObject* parent = nullptr);
	~ExternalPlcBridge() override;

	void setNetwork(IoSignalNetworkService* network);
	void setLinkConfig(const ExternalPlcLinkConfig& config);
	ExternalPlcLinkConfig linkConfig() const { return m_link; }
	void setSignalMap(const QVector<PlcSignalMapEntry>& map);
	const QVector<PlcSignalMapEntry>& signalMap() const { return m_map; }

	bool connectPlc(QString* err = nullptr);
	void disconnectPlc();
	bool isPlcConnected() const { return m_connected; }

	/// 仿真 tick 尾调用：应用缓存 in；投递 out；触发后台一轮同步
	void onAfterIoPhase();

signals:
	void plcConnectionChanged(bool connected);
	void plcWarning(const QString& message);

private slots:
	void onWorkerFinished(bool ok, const QString& error, const QHash<QString, bool>& inValues);
	void onWorkerDisconnected();

private:
	struct PendingOut
	{
		QString plcTag;
		bool value = false;
	};

	void ensureWorker();
	void stopWorker();
	QString mapKey(const QString& ownerId, const QString& signal) const;

	IoSignalNetworkService* m_network = nullptr;
	ExternalPlcLinkConfig m_link;
	QVector<PlcSignalMapEntry> m_map;
	bool m_connected = false;
	bool m_syncBusy = false;
	bool m_disconnectWarned = false;

	QHash<QString, bool> m_cachedIn;
	QVector<PendingOut> m_pendingOut;

	QThread* m_thread = nullptr;
	QObject* m_worker = nullptr;
};

#endif // ROBOTWIDGET_EXTERNALPLCBRIDGE_H
