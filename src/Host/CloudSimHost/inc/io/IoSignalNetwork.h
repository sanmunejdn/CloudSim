#ifndef CLOUDSIMHOST_IOSIGNALNETWORK_H
#define CLOUDSIMHOST_IOSIGNALNETWORK_H

/// @file IoSignalNetwork.h
/// @brief Headless/Web 多 Owner IO 网：与桌面 ioSignalNetwork 侧车同形

#include "cloudsim_host_global.h"

#include "IRobotIoSink.h"
#include "NamedSignalTable.h"

#include <QHash>
#include <QJsonObject>
#include <QObject>
#include <QString>
#include <QStringList>
#include <QVector>

namespace cloudsim::host
{
class DocumentHost;

enum class IoSignalOwnerKind : int
{
	Robot = 0,
	Device = 1
};

struct CLOUDSIM_HOST_EXPORT IoSignalWire
{
	QString id;
	QString fromOwnerId;
	QString fromSignal;
	QString toOwnerId;
	QString toSignal;
};

/// 每 Owner 一张 NamedSignalTable + 运行时；DO→DI 连线传播
class CLOUDSIM_HOST_EXPORT IoSignalNetwork : public QObject, public IRobotIoSink
{
	Q_OBJECT

public:
	explicit IoSignalNetwork(QObject* parent = nullptr);
	~IoSignalNetwork() override;

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

	/// 指令属性 / 旧 /api/io/signals：优先第一台机器人，否则占位 Owner
	QString primaryRobotOwnerId() const;
	RobotIo::NamedSignalTable& primaryTable();
	const RobotIo::NamedSignalTable& primaryTable() const;

	double canvasX(const QString& ownerId) const;
	double canvasY(const QString& ownerId) const;
	void setCanvasPos(const QString& ownerId, double x, double y);

	const QVector<IoSignalWire>& wires() const { return m_wires; }
	bool addWire(const IoSignalWire& wire, QString* err = nullptr);
	bool removeWire(const QString& wireId);
	void removeWiresTouchingSignal(const QString& ownerId, const QString& signalName);

	void propagateFrom(const QString& ownerId);
	void propagateAll();

	void syncOwnersFromDocument(DocumentHost& host);
	void flushDeviceTablesToDocument(DocumentHost& host);

	QJsonObject toProjectJson() const;
	bool fromProjectJson(const QJsonObject& root, QString* err = nullptr);
	/// 旧单表 ioSignals → 写入主机器人 Owner
	bool importLegacyFlatSignals(const QJsonObject& signalsWrap, QString* err = nullptr);

	QJsonObject ownerSignalsPayloadWithRuntime(const QString& ownerId) const;
	QJsonObject networkPayloadWithRuntime() const;

	bool setRuntime(const QString& ownerId, const QString& kind, int port, const QString& valueText, bool hasForced,
					bool forced, QString* err = nullptr);
	void resetRuntime(const QString& ownerId = QString(), bool keepForcedDi = false);

	// IRobotIoSink：作用于 primary robot（程序 Run）
	void setDigitalOutput(int port, bool value) override;
	void setAnalogOutput(int port, double value) override;
	bool getDigitalInput(int port, bool* outValue) const override;
	int resolveNamedPort(const std::string& signalName, int fallbackPort) const override;

signals:
	void networkChanged();
	void ownerIoChanged(const QString& ownerId);

private:
	struct OwnerRuntime
	{
		QHash<int, bool> di;
		QHash<int, bool> digitalOut;
		QHash<int, double> ai;
		QHash<int, double> ao;
		QHash<int, bool> diForced;
	};

	struct OwnerState
	{
		IoSignalOwnerKind kind = IoSignalOwnerKind::Robot;
		QString displayName;
		RobotIo::NamedSignalTable table;
		OwnerRuntime runtime;
		double canvasX = 120.0;
		double canvasY = 120.0;
	};

	OwnerState* mutableOwner(const QString& ownerId);
	const OwnerState* owner(const QString& ownerId) const;
	static QString makeWireId();
	bool validateWire(const IoSignalWire& wire, QString* err) const;
	void resetOwnerRuntime(OwnerState& st, bool keepForcedDi);
	static bool parseBoolText(const QString& t);

	QHash<QString, OwnerState*> m_owners;
	QVector<IoSignalWire> m_wires;
	bool m_propagating = false;
	mutable RobotIo::NamedSignalTable m_emptyFallback;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_IOSIGNALNETWORK_H
