#ifndef ROBOTWIDGET_IOSIGNALNETWORKSERVICE_H
#define ROBOTWIDGET_IOSIGNALNETWORKSERVICE_H

/// @file IoSignalNetworkService.h
/// @brief 每 Owner 信号表/sink + DO→DI 接线图

#include "robotwidget_global.h"

#include "IRobotIoSink.h"
#include "NamedSignalTable.h"

#include <QHash>
#include <QJsonObject>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>
#include <memory>

class IRobotDocumentHost;
class NamedSignalIoSink;

enum class IoSignalOwnerKind : int
{
	Robot = 0,
	Device = 1
};

struct ROBOTWIDGET_EXPORT IoSignalWire
{
	QString id;
	QString fromOwnerId;
	QString fromSignal;
	QString toOwnerId;
	QString toSignal;
};

class ROBOTWIDGET_EXPORT IoSignalNetworkService : public QObject
{
	Q_OBJECT

public:
	explicit IoSignalNetworkService(QObject* parent = nullptr);
	~IoSignalNetworkService() override;

	void clear();
	void ensureOwner(IoSignalOwnerKind kind, const QString& ownerId, const QString& displayName);
	void removeOwner(const QString& ownerId);
	QStringList ownerIds() const;
	bool hasOwner(const QString& ownerId) const;
	IoSignalOwnerKind ownerKind(const QString& ownerId) const;
	QString displayName(const QString& ownerId) const;
	void setDisplayName(const QString& ownerId, const QString& displayName);

	RobotIo::NamedSignalTable* table(const QString& ownerId);
	const RobotIo::NamedSignalTable* table(const QString& ownerId) const;
	NamedSignalIoSink* sink(const QString& ownerId);

	double canvasX(const QString& ownerId) const;
	double canvasY(const QString& ownerId) const;
	void setCanvasPos(const QString& ownerId, double x, double y);

	const QVector<IoSignalWire>& wires() const { return m_wires; }
	bool addWire(const IoSignalWire& wire, QString* err = nullptr);
	bool removeWire(const QString& wireId);
	void setWires(QVector<IoSignalWire> wires);
	void removeWiresTouchingSignal(const QString& ownerId, const QString& signalName);

	void propagateFrom(const QString& ownerId);
	void propagateAll();

	void syncOwnersFromDocument(IRobotDocumentHost* doc);
	void flushDeviceTablesToDocument(IRobotDocumentHost* doc);

	QJsonObject toProjectJson() const;
	bool fromProjectJson(const QJsonObject& root, QString* err = nullptr);

	void setIoSinkBackend(RobotIoSinkBackend backend);

signals:
	void networkChanged();
	void ownerIoChanged(const QString& ownerId);

private slots:
	void onOwnerSinkChanged();

private:
	struct OwnerState
	{
		IoSignalOwnerKind kind = IoSignalOwnerKind::Robot;
		QString displayName;
		RobotIo::NamedSignalTable table;
		NamedSignalIoSink* sink = nullptr;
		double canvasX = 120.0;
		double canvasY = 120.0;
	};

	OwnerState* mutableOwner(const QString& ownerId);
	const OwnerState* owner(const QString& ownerId) const;
	void connectSink(NamedSignalIoSink* sink);
	static QString makeWireId();
	bool validateWire(const IoSignalWire& wire, QString* err) const;

	QHash<QString, OwnerState*> m_owners;
	QVector<IoSignalWire> m_wires;
	bool m_propagating = false;
	RobotIoSinkBackend m_backend = RobotIoSinkBackend::Simulation;
};

#endif // ROBOTWIDGET_IOSIGNALNETWORKSERVICE_H
