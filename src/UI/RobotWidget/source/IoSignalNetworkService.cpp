/// @file IoSignalNetworkService.cpp
/// @brief IO 信号网络（桌面）

#include "IoSignalNetworkService.h"

#include "BackendTypeIds.h"
#include "CustomDeviceBackendData.h"
#include "IRobotDocumentHost.h"
#include "NamedSignalIoSink.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QSet>
#include <QSignalBlocker>
#include <QVariant>

#include <atomic>

namespace
{
QJsonArray signalsArrayFromTable(const RobotIo::NamedSignalTable& table)
{
	const nlohmann::json tj = table.toJson();
	if (!tj.contains("signals") || !tj["signals"].is_array())
	{
		return {};
	}
	const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(tj["signals"].dump()));
	return doc.isArray() ? doc.array() : QJsonArray();
}

bool loadTableFromSignalsValue(RobotIo::NamedSignalTable& table, const nlohmann::json& signalsOrWrap, std::string* err)
{
	return table.fromJson(signalsOrWrap, err);
}
} // namespace

IoSignalNetworkService::IoSignalNetworkService(QObject* parent) : QObject(parent) {}

IoSignalNetworkService::~IoSignalNetworkService()
{
	const QSignalBlocker blocker(this);
	clear();
}

void IoSignalNetworkService::clear()
{
	for (auto it = m_owners.begin(); it != m_owners.end(); ++it)
	{
		OwnerState* st = it.value();
		if (!st)
		{
			continue;
		}
		if (st->sink)
		{
			disconnect(st->sink, nullptr, this, nullptr);
			delete st->sink;
			st->sink = nullptr;
		}
		delete st;
	}
	m_owners.clear();
	m_wires.clear();
	emit networkChanged();
}

IoSignalNetworkService::OwnerState* IoSignalNetworkService::mutableOwner(const QString& ownerId)
{
	auto it = m_owners.find(ownerId);
	return it == m_owners.end() ? nullptr : it.value();
}

const IoSignalNetworkService::OwnerState* IoSignalNetworkService::owner(const QString& ownerId) const
{
	auto it = m_owners.constFind(ownerId);
	return it == m_owners.constEnd() ? nullptr : it.value();
}

void IoSignalNetworkService::connectSink(NamedSignalIoSink* sink)
{
	if (!sink)
	{
		return;
	}
	connect(sink, &NamedSignalIoSink::ioValuesChanged, this, &IoSignalNetworkService::onOwnerSinkChanged,
			Qt::UniqueConnection);
}

void IoSignalNetworkService::ensureOwner(const IoSignalOwnerKind kind, const QString& ownerId,
										 const QString& displayName)
{
	if (ownerId.isEmpty())
	{
		return;
	}
	if (OwnerState* st = mutableOwner(ownerId))
	{
		st->kind = kind;
		if (!displayName.isEmpty())
		{
			st->displayName = displayName;
		}
		return;
	}
	auto* created = new OwnerState();
	created->kind = kind;
	created->displayName = displayName.isEmpty() ? ownerId : displayName;
	created->sink = new NamedSignalIoSink(this);
	created->sink->setProperty("ownerId", ownerId);
	created->sink->setSignalTable(&created->table);
	created->sink->setIoSinkBackend(m_backend);
	created->sink->resetRuntimeFromTable(false);
	connectSink(created->sink);
	const int n = m_owners.size();
	created->canvasX = 120.0 + (n % 4) * 220.0;
	created->canvasY = 100.0 + (n / 4) * 180.0;
	m_owners.insert(ownerId, created);
	emit networkChanged();
}

void IoSignalNetworkService::removeOwner(const QString& ownerId)
{
	OwnerState* st = mutableOwner(ownerId);
	if (!st)
	{
		return;
	}
	for (int i = m_wires.size() - 1; i >= 0; --i)
	{
		if (m_wires[i].fromOwnerId == ownerId || m_wires[i].toOwnerId == ownerId)
		{
			m_wires.removeAt(i);
		}
	}
	if (st->sink)
	{
		disconnect(st->sink, nullptr, this, nullptr);
		delete st->sink;
		st->sink = nullptr;
	}
	m_owners.remove(ownerId);
	delete st;
	emit networkChanged();
}

QStringList IoSignalNetworkService::ownerIds() const
{
	return m_owners.keys();
}

bool IoSignalNetworkService::hasOwner(const QString& ownerId) const
{
	return m_owners.contains(ownerId);
}

IoSignalOwnerKind IoSignalNetworkService::ownerKind(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? st->kind : IoSignalOwnerKind::Robot;
}

QString IoSignalNetworkService::displayName(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? st->displayName : QString();
}

void IoSignalNetworkService::setDisplayName(const QString& ownerId, const QString& displayName)
{
	if (OwnerState* st = mutableOwner(ownerId))
	{
		st->displayName = displayName;
	}
}

RobotIo::NamedSignalTable* IoSignalNetworkService::table(const QString& ownerId)
{
	OwnerState* st = mutableOwner(ownerId);
	return st ? &st->table : nullptr;
}

const RobotIo::NamedSignalTable* IoSignalNetworkService::table(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? &st->table : nullptr;
}

NamedSignalIoSink* IoSignalNetworkService::sink(const QString& ownerId)
{
	OwnerState* st = mutableOwner(ownerId);
	return st ? st->sink : nullptr;
}

double IoSignalNetworkService::canvasX(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? st->canvasX : 0.0;
}

double IoSignalNetworkService::canvasY(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? st->canvasY : 0.0;
}

void IoSignalNetworkService::setCanvasPos(const QString& ownerId, const double x, const double y)
{
	if (OwnerState* st = mutableOwner(ownerId))
	{
		st->canvasX = x;
		st->canvasY = y;
	}
}

QString IoSignalNetworkService::makeWireId()
{
	static std::atomic<unsigned long long> s{1ULL};
	return QStringLiteral("WIRE_%1").arg(s.fetch_add(1ULL));
}

bool IoSignalNetworkService::validateWire(const IoSignalWire& wire, QString* err) const
{
	if (wire.fromOwnerId.isEmpty() || wire.toOwnerId.isEmpty() || wire.fromSignal.isEmpty() || wire.toSignal.isEmpty())
	{
		if (err)
		{
			*err = QStringLiteral("接线端点不完整。");
		}
		return false;
	}
	if (wire.fromOwnerId == wire.toOwnerId && wire.fromSignal == wire.toSignal)
	{
		if (err)
		{
			*err = QStringLiteral("不能接到同一信号。");
		}
		return false;
	}
	const OwnerState* from = owner(wire.fromOwnerId);
	const OwnerState* to = owner(wire.toOwnerId);
	if (!from || !to)
	{
		if (err)
		{
			*err = QStringLiteral("接线引用了不存在的 Owner。");
		}
		return false;
	}
	const RobotIo::SignalDef* fromDef = from->table.findByName(wire.fromSignal.toStdString());
	const RobotIo::SignalDef* toDef = to->table.findByName(wire.toSignal.toStdString());
	if (!fromDef || fromDef->kind != RobotIo::SignalKind::DO)
	{
		if (err)
		{
			*err = QStringLiteral("源必须是 DO。");
		}
		return false;
	}
	if (!toDef || toDef->kind != RobotIo::SignalKind::DI)
	{
		if (err)
		{
			*err = QStringLiteral("目标必须是 DI。");
		}
		return false;
	}
	for (const IoSignalWire& w : m_wires)
	{
		if (w.id == wire.id)
		{
			continue;
		}
		if (w.toOwnerId == wire.toOwnerId && w.toSignal == wire.toSignal)
		{
			if (err)
			{
				*err = QStringLiteral("同一 DI 禁止多入线。");
			}
			return false;
		}
	}
	return true;
}

bool IoSignalNetworkService::addWire(const IoSignalWire& wire, QString* err)
{
	IoSignalWire w = wire;
	if (w.id.isEmpty())
	{
		w.id = makeWireId();
	}
	if (!validateWire(w, err))
	{
		return false;
	}
	for (const IoSignalWire& existing : m_wires)
	{
		if (existing.fromOwnerId == w.fromOwnerId && existing.fromSignal == w.fromSignal &&
			existing.toOwnerId == w.toOwnerId && existing.toSignal == w.toSignal)
		{
			return true;
		}
	}
	m_wires.push_back(w);
	propagateFrom(w.fromOwnerId);
	emit networkChanged();
	return true;
}

bool IoSignalNetworkService::removeWire(const QString& wireId)
{
	for (int i = 0; i < m_wires.size(); ++i)
	{
		if (m_wires[i].id == wireId)
		{
			m_wires.removeAt(i);
			emit networkChanged();
			return true;
		}
	}
	return false;
}

void IoSignalNetworkService::setWires(QVector<IoSignalWire> wires)
{
	m_wires = std::move(wires);
	emit networkChanged();
	propagateAll();
}

void IoSignalNetworkService::removeWiresTouchingSignal(const QString& ownerId, const QString& signalName)
{
	bool changed = false;
	for (int i = m_wires.size() - 1; i >= 0; --i)
	{
		const IoSignalWire& w = m_wires[i];
		if ((w.fromOwnerId == ownerId && w.fromSignal == signalName) ||
			(w.toOwnerId == ownerId && w.toSignal == signalName))
		{
			m_wires.removeAt(i);
			changed = true;
		}
	}
	if (changed)
	{
		emit networkChanged();
	}
}

void IoSignalNetworkService::propagateFrom(const QString& ownerId)
{
	if (m_propagating || ownerId.isEmpty())
	{
		return;
	}
	const OwnerState* from = owner(ownerId);
	if (!from || !from->sink)
	{
		return;
	}
	m_propagating = true;
	QSet<QString> changedTargets;
	for (const IoSignalWire& w : m_wires)
	{
		if (w.fromOwnerId != ownerId)
		{
			continue;
		}
		OwnerState* to = mutableOwner(w.toOwnerId);
		if (!to || !to->sink)
		{
			continue;
		}
		const RobotIo::SignalDef* fromDef = from->table.findByName(w.fromSignal.toStdString());
		const RobotIo::SignalDef* toDef = to->table.findByName(w.toSignal.toStdString());
		if (!fromDef || !toDef)
		{
			continue;
		}
		bool value = false;
		if (!from->sink->getDigitalOutput(fromDef->port, &value))
		{
			continue;
		}
		if (to->sink->isDigitalInputForced(toDef->port))
		{
			continue;
		}
		bool cur = false;
		(void)to->sink->getDigitalInput(toDef->port, &cur);
		if (cur == value)
		{
			continue;
		}
		to->sink->setDigitalInput(toDef->port, value);
		changedTargets.insert(w.toOwnerId);
	}
	m_propagating = false;
	// 传播中写入目标 DI 时回调被 m_propagating 吞掉，须补发否则设备姿态绑定收不到上升沿
	for (const QString& tid : changedTargets)
	{
		emit ownerIoChanged(tid);
	}
}

void IoSignalNetworkService::propagateAll()
{
	for (const QString& id : ownerIds())
	{
		propagateFrom(id);
	}
}

void IoSignalNetworkService::onOwnerSinkChanged()
{
	if (m_propagating)
	{
		return;
	}
	auto* s = qobject_cast<NamedSignalIoSink*>(sender());
	if (!s)
	{
		return;
	}
	const QString ownerId = s->property("ownerId").toString();
	if (ownerId.isEmpty())
	{
		return;
	}
	propagateFrom(ownerId);
	emit ownerIoChanged(ownerId);
}

void IoSignalNetworkService::syncOwnersFromDocument(IRobotDocumentHost* doc)
{
	if (!doc)
	{
		return;
	}
	QSet<QString> live;
	const int n = doc->robotKinematicInstanceCount();
	for (int i = 0; i < n; ++i)
	{
		const QString id = doc->robotSceneBackendIdForInstance(i);
		if (id.isEmpty())
		{
			continue;
		}
		live.insert(id);
		ensureOwner(IoSignalOwnerKind::Robot, id, doc->robotDisplayLabelForInstance(i));
	}
	for (const QString& id : doc->documentData().findByClassName(QString::fromUtf8(backend_type::kClassCustomDevice)))
	{
		if (id.isEmpty())
		{
			continue;
		}
		live.insert(id);
		QString label = doc->documentData().displayName(id);
		if (label.isEmpty())
		{
			label = id;
		}
		ensureOwner(IoSignalOwnerKind::Device, id, label);
		OwnerState* st = mutableOwner(id);
		const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(id.toStdString()));
		if (!st || !device)
		{
			continue;
		}
		std::string e;
		(void)loadTableFromSignalsValue(st->table, device->ioSignalsJson(), &e);
		if (st->sink)
		{
			st->sink->setSignalTable(&st->table);
			st->sink->resetRuntimeFromTable(true);
		}
	}
	for (const QString& id : ownerIds())
	{
		if (!live.contains(id))
		{
			removeOwner(id);
		}
	}
	propagateAll();
	emit networkChanged();
}

void IoSignalNetworkService::flushDeviceTablesToDocument(IRobotDocumentHost* doc)
{
	if (!doc)
	{
		return;
	}
	for (auto it = m_owners.begin(); it != m_owners.end(); ++it)
	{
		OwnerState* st = it.value();
		if (!st || st->kind != IoSignalOwnerKind::Device)
		{
			continue;
		}
		const auto device =
			std::dynamic_pointer_cast<CustomDeviceBackendData>(doc->findObject(it.key().toStdString()));
		if (!device)
		{
			continue;
		}
		device->setIoSignalsJson(st->table.toJson());
	}
}

QJsonObject IoSignalNetworkService::toProjectJson() const
{
	QJsonObject root;
	QJsonObject owners;
	for (auto it = m_owners.constBegin(); it != m_owners.constEnd(); ++it)
	{
		const OwnerState* st = it.value();
		if (!st)
		{
			continue;
		}
		QJsonObject o;
		o.insert(QStringLiteral("kind"),
				 st->kind == IoSignalOwnerKind::Device ? QStringLiteral("device") : QStringLiteral("robot"));
		o.insert(QStringLiteral("displayName"), st->displayName);
		o.insert(QStringLiteral("canvasX"), st->canvasX);
		o.insert(QStringLiteral("canvasY"), st->canvasY);
		if (st->kind == IoSignalOwnerKind::Robot)
		{
			o.insert(QStringLiteral("signals"), signalsArrayFromTable(st->table));
		}
		owners.insert(it.key(), o);
	}
	root.insert(QStringLiteral("owners"), owners);
	QJsonArray wires;
	for (const IoSignalWire& w : m_wires)
	{
		QJsonObject j;
		j.insert(QStringLiteral("id"), w.id);
		j.insert(QStringLiteral("fromOwnerId"), w.fromOwnerId);
		j.insert(QStringLiteral("fromSignal"), w.fromSignal);
		j.insert(QStringLiteral("toOwnerId"), w.toOwnerId);
		j.insert(QStringLiteral("toSignal"), w.toSignal);
		wires.append(j);
	}
	root.insert(QStringLiteral("wires"), wires);
	return root;
}

bool IoSignalNetworkService::fromProjectJson(const QJsonObject& root, QString* err)
{
	// 加载中抑制 networkChanged，避免 UI 在 clear 删 sink 后立刻 disconnect 悬空指针
	{
		const QSignalBlocker blocker(this);
		clear();
		const QJsonObject owners = root.value(QStringLiteral("owners")).toObject();
		for (auto it = owners.begin(); it != owners.end(); ++it)
		{
			const QJsonObject o = it.value().toObject();
			const QString kindStr = o.value(QStringLiteral("kind")).toString(QStringLiteral("robot"));
			const IoSignalOwnerKind kind =
				kindStr == QLatin1String("device") ? IoSignalOwnerKind::Device : IoSignalOwnerKind::Robot;
			ensureOwner(kind, it.key(), o.value(QStringLiteral("displayName")).toString(it.key()));
			OwnerState* st = mutableOwner(it.key());
			if (!st)
			{
				continue;
			}
			st->canvasX = o.value(QStringLiteral("canvasX")).toDouble(st->canvasX);
			st->canvasY = o.value(QStringLiteral("canvasY")).toDouble(st->canvasY);
			if (kind == IoSignalOwnerKind::Robot && o.contains(QStringLiteral("signals")))
			{
				nlohmann::json wrap = nlohmann::json::object();
				const QByteArray raw =
					QJsonDocument(o.value(QStringLiteral("signals")).toArray()).toJson(QJsonDocument::Compact);
				wrap["signals"] = nlohmann::json::parse(raw.constData(), nullptr, false);
				if (wrap["signals"].is_discarded())
				{
					wrap["signals"] = nlohmann::json::array();
				}
				std::string e;
				if (!st->table.fromJson(wrap, &e))
				{
					if (err)
					{
						*err = QString::fromStdString(e);
					}
					return false;
				}
				if (st->sink)
				{
					st->sink->setSignalTable(&st->table);
					st->sink->resetRuntimeFromTable(false);
				}
			}
		}
		QVector<IoSignalWire> loaded;
		for (const QJsonValue& v : root.value(QStringLiteral("wires")).toArray())
		{
			const QJsonObject j = v.toObject();
			IoSignalWire w;
			w.id = j.value(QStringLiteral("id")).toString();
			w.fromOwnerId = j.value(QStringLiteral("fromOwnerId")).toString();
			w.fromSignal = j.value(QStringLiteral("fromSignal")).toString();
			w.toOwnerId = j.value(QStringLiteral("toOwnerId")).toString();
			w.toSignal = j.value(QStringLiteral("toSignal")).toString();
			if (w.id.isEmpty())
			{
				w.id = makeWireId();
			}
			loaded.push_back(w);
		}
		m_wires = loaded;
	}
	emit networkChanged();
	return true;
}

void IoSignalNetworkService::setIoSinkBackend(const RobotIoSinkBackend backend)
{
	m_backend = backend;
	for (auto it = m_owners.begin(); it != m_owners.end(); ++it)
	{
		if (it.value() && it.value()->sink)
		{
			it.value()->sink->setIoSinkBackend(backend);
		}
	}
}
