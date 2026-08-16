/// @file IoSignalNetwork.cpp
/// @brief Headless/Web IoSignalNetwork 实现

#include "IoSignalNetwork.h"

#include "BackendTypeIds.h"
#include "CustomDeviceBackendData.h"
#include "DocumentHost.h"
#include "HeadlessRobotContext.h"

#include <QJsonArray>
#include <QJsonDocument>
#include <QSet>

#include <atomic>

namespace cloudsim::host
{
namespace
{
QJsonArray signalsArrayFromTable(const RobotIo::NamedSignalTable& table)
{
	const nlohmann::json tj = table.toJson();
	if (!tj.contains("signals") || !tj["signals"].is_array())
		return {};
	const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(tj["signals"].dump()));
	return doc.isArray() ? doc.array() : QJsonArray();
}

constexpr const char* kLegacyOwnerId = "__legacy_robot__";
} // namespace

IoSignalNetwork::IoSignalNetwork(QObject* parent) : QObject(parent) {}

IoSignalNetwork::~IoSignalNetwork()
{
	clear();
}

void IoSignalNetwork::clear()
{
	for (auto it = m_owners.begin(); it != m_owners.end(); ++it)
		delete it.value();
	m_owners.clear();
	m_wires.clear();
	emit networkChanged();
}

IoSignalNetwork::OwnerState* IoSignalNetwork::mutableOwner(const QString& ownerId)
{
	auto it = m_owners.find(ownerId);
	return it == m_owners.end() ? nullptr : it.value();
}

const IoSignalNetwork::OwnerState* IoSignalNetwork::owner(const QString& ownerId) const
{
	auto it = m_owners.constFind(ownerId);
	return it == m_owners.constEnd() ? nullptr : it.value();
}

void IoSignalNetwork::ensureOwner(const IoSignalOwnerKind kind, const QString& ownerId, const QString& displayName)
{
	if (ownerId.isEmpty())
		return;
	if (OwnerState* st = mutableOwner(ownerId))
	{
		st->kind = kind;
		if (!displayName.isEmpty())
			st->displayName = displayName;
		return;
	}
	auto* created = new OwnerState();
	created->kind = kind;
	created->displayName = displayName.isEmpty() ? ownerId : displayName;
	resetOwnerRuntime(*created, false);
	const int n = m_owners.size();
	created->canvasX = 120.0 + (n % 4) * 220.0;
	created->canvasY = 100.0 + (n / 4) * 180.0;
	m_owners.insert(ownerId, created);
	emit networkChanged();
}

void IoSignalNetwork::removeOwner(const QString& ownerId)
{
	OwnerState* st = mutableOwner(ownerId);
	if (!st)
		return;
	for (int i = m_wires.size() - 1; i >= 0; --i)
	{
		if (m_wires[i].fromOwnerId == ownerId || m_wires[i].toOwnerId == ownerId)
			m_wires.removeAt(i);
	}
	m_owners.remove(ownerId);
	delete st;
	emit networkChanged();
}

QStringList IoSignalNetwork::ownerIds() const
{
	return m_owners.keys();
}

bool IoSignalNetwork::hasOwner(const QString& ownerId) const
{
	return m_owners.contains(ownerId);
}

IoSignalOwnerKind IoSignalNetwork::ownerKind(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? st->kind : IoSignalOwnerKind::Robot;
}

QString IoSignalNetwork::displayName(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? st->displayName : QString();
}

void IoSignalNetwork::setDisplayName(const QString& ownerId, const QString& displayName)
{
	if (OwnerState* st = mutableOwner(ownerId))
		st->displayName = displayName;
}

RobotIo::NamedSignalTable* IoSignalNetwork::table(const QString& ownerId)
{
	OwnerState* st = mutableOwner(ownerId);
	return st ? &st->table : nullptr;
}

const RobotIo::NamedSignalTable* IoSignalNetwork::table(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? &st->table : nullptr;
}

QString IoSignalNetwork::primaryRobotOwnerId() const
{
	for (auto it = m_owners.constBegin(); it != m_owners.constEnd(); ++it)
	{
		if (it.value() && it.value()->kind == IoSignalOwnerKind::Robot && it.key() != QLatin1String(kLegacyOwnerId))
			return it.key();
	}
	if (m_owners.contains(QLatin1String(kLegacyOwnerId)))
		return QString::fromLatin1(kLegacyOwnerId);
	for (auto it = m_owners.constBegin(); it != m_owners.constEnd(); ++it)
	{
		if (it.value() && it.value()->kind == IoSignalOwnerKind::Robot)
			return it.key();
	}
	return {};
}

RobotIo::NamedSignalTable& IoSignalNetwork::primaryTable()
{
	const QString id = primaryRobotOwnerId();
	if (id.isEmpty())
	{
		ensureOwner(IoSignalOwnerKind::Robot, QString::fromLatin1(kLegacyOwnerId), QStringLiteral("Robot"));
		return mutableOwner(QString::fromLatin1(kLegacyOwnerId))->table;
	}
	return *table(id);
}

const RobotIo::NamedSignalTable& IoSignalNetwork::primaryTable() const
{
	const QString id = primaryRobotOwnerId();
	if (id.isEmpty())
		return m_emptyFallback;
	return *table(id);
}

double IoSignalNetwork::canvasX(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? st->canvasX : 0.0;
}

double IoSignalNetwork::canvasY(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	return st ? st->canvasY : 0.0;
}

void IoSignalNetwork::setCanvasPos(const QString& ownerId, const double x, const double y)
{
	if (OwnerState* st = mutableOwner(ownerId))
	{
		st->canvasX = x;
		st->canvasY = y;
	}
}

QString IoSignalNetwork::makeWireId()
{
	static std::atomic<unsigned long long> s{1ULL};
	return QStringLiteral("WIRE_%1").arg(s.fetch_add(1ULL));
}

bool IoSignalNetwork::validateWire(const IoSignalWire& wire, QString* err) const
{
	if (wire.fromOwnerId.isEmpty() || wire.toOwnerId.isEmpty() || wire.fromSignal.isEmpty() || wire.toSignal.isEmpty())
	{
		if (err)
			*err = QStringLiteral("接线端点不完整。");
		return false;
	}
	if (wire.fromOwnerId == wire.toOwnerId && wire.fromSignal == wire.toSignal)
	{
		if (err)
			*err = QStringLiteral("不能接到同一信号。");
		return false;
	}
	const OwnerState* from = owner(wire.fromOwnerId);
	const OwnerState* to = owner(wire.toOwnerId);
	if (!from || !to)
	{
		if (err)
			*err = QStringLiteral("接线引用了不存在的 Owner。");
		return false;
	}
	const RobotIo::SignalDef* fromDef = from->table.findByName(wire.fromSignal.toStdString());
	const RobotIo::SignalDef* toDef = to->table.findByName(wire.toSignal.toStdString());
	if (!fromDef || fromDef->kind != RobotIo::SignalKind::DO)
	{
		if (err)
			*err = QStringLiteral("源必须是 DO。");
		return false;
	}
	if (!toDef || toDef->kind != RobotIo::SignalKind::DI)
	{
		if (err)
			*err = QStringLiteral("目标必须是 DI。");
		return false;
	}
	for (const IoSignalWire& w : m_wires)
	{
		if (w.id == wire.id)
			continue;
		if (w.toOwnerId == wire.toOwnerId && w.toSignal == wire.toSignal)
		{
			if (err)
				*err = QStringLiteral("同一 DI 禁止多入线。");
			return false;
		}
	}
	return true;
}

bool IoSignalNetwork::addWire(const IoSignalWire& wire, QString* err)
{
	IoSignalWire w = wire;
	if (w.id.isEmpty())
		w.id = makeWireId();
	if (!validateWire(w, err))
		return false;
	for (const IoSignalWire& existing : m_wires)
	{
		if (existing.fromOwnerId == w.fromOwnerId && existing.fromSignal == w.fromSignal &&
			existing.toOwnerId == w.toOwnerId && existing.toSignal == w.toSignal)
			return true;
	}
	m_wires.push_back(w);
	propagateFrom(w.fromOwnerId);
	emit networkChanged();
	return true;
}

bool IoSignalNetwork::removeWire(const QString& wireId)
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

void IoSignalNetwork::removeWiresTouchingSignal(const QString& ownerId, const QString& signalName)
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
		emit networkChanged();
}

void IoSignalNetwork::resetOwnerRuntime(OwnerState& st, const bool keepForcedDi)
{
	QHash<int, bool> kept;
	if (keepForcedDi)
		kept = st.runtime.diForced;
	st.runtime = OwnerRuntime{};
	if (keepForcedDi)
		st.runtime.diForced = kept;
	for (const RobotIo::SignalDef& s : st.table.entries())
	{
		switch (s.kind)
		{
		case RobotIo::SignalKind::DI:
			st.runtime.di.insert(s.port, st.runtime.diForced.contains(s.port) ? st.runtime.diForced.value(s.port)
																			  : s.defaultBool);
			break;
		case RobotIo::SignalKind::DO:
			st.runtime.digitalOut.insert(s.port, s.defaultBool);
			break;
		case RobotIo::SignalKind::AI:
			st.runtime.ai.insert(s.port, s.defaultAnalog);
			break;
		case RobotIo::SignalKind::AO:
			st.runtime.ao.insert(s.port, s.defaultAnalog);
			break;
		}
	}
}

void IoSignalNetwork::propagateFrom(const QString& ownerId)
{
	if (m_propagating || ownerId.isEmpty())
		return;
	const OwnerState* from = owner(ownerId);
	if (!from)
		return;
	m_propagating = true;
	QSet<QString> changedTargets;
	for (const IoSignalWire& w : m_wires)
	{
		if (w.fromOwnerId != ownerId)
			continue;
		OwnerState* to = mutableOwner(w.toOwnerId);
		if (!to)
			continue;
		const RobotIo::SignalDef* fromDef = from->table.findByName(w.fromSignal.toStdString());
		const RobotIo::SignalDef* toDef = to->table.findByName(w.toSignal.toStdString());
		if (!fromDef || !toDef)
			continue;
		const bool value = from->runtime.digitalOut.value(fromDef->port, fromDef->defaultBool);
		if (to->runtime.diForced.contains(toDef->port))
			continue;
		const bool cur = to->runtime.di.value(toDef->port, toDef->defaultBool);
		if (cur == value)
			continue;
		to->runtime.di.insert(toDef->port, value);
		changedTargets.insert(w.toOwnerId);
	}
	m_propagating = false;
	for (const QString& tid : changedTargets)
		emit ownerIoChanged(tid);
}

void IoSignalNetwork::propagateAll()
{
	for (const QString& id : ownerIds())
		propagateFrom(id);
}

void IoSignalNetwork::syncOwnersFromDocument(DocumentHost& host)
{
	QSet<QString> live;
	if (HeadlessRobotContext* hrc = host.headlessRobotContext())
	{
		for (const auto& info : hrc->listInstances())
		{
			if (info.sceneRootBackendId.isEmpty())
				continue;
			live.insert(info.sceneRootBackendId);
			ensureOwner(IoSignalOwnerKind::Robot, info.sceneRootBackendId, info.label);
		}
	}
	for (const auto& obj : host.listObjects())
	{
		if (!obj || obj->className() != backend_type::kClassCustomDevice)
			continue;
		const QString id = QString::fromStdString(obj->id());
		live.insert(id);
		QString label = QString::fromStdString(obj->name());
		if (label.isEmpty())
			label = id;
		ensureOwner(IoSignalOwnerKind::Device, id, label);
		OwnerState* st = mutableOwner(id);
		const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(obj);
		if (!st || !device)
			continue;
		std::string e;
		(void)st->table.fromJson(device->ioSignalsJson(), &e);
		// 勿在每次 sync/GET 时 reset：会抹掉 DI 运行时并弄乱上升沿检测
	}
	// 把占位 legacy 表迁到第一台真机机器人
	if (OwnerState* legacy = mutableOwner(QString::fromLatin1(kLegacyOwnerId)))
	{
		const QString primary = primaryRobotOwnerId();
		if (!primary.isEmpty() && primary != QLatin1String(kLegacyOwnerId))
		{
			if (OwnerState* dest = mutableOwner(primary))
			{
				if (dest->table.entries().empty() && !legacy->table.entries().empty())
				{
					dest->table = legacy->table;
					dest->runtime = legacy->runtime;
				}
			}
			removeOwner(QString::fromLatin1(kLegacyOwnerId));
		}
		else
		{
			live.insert(QString::fromLatin1(kLegacyOwnerId));
		}
	}
	for (const QString& id : ownerIds())
	{
		if (!live.contains(id))
			removeOwner(id);
	}
	propagateAll();
	emit networkChanged();
}

void IoSignalNetwork::flushDeviceTablesToDocument(DocumentHost& host)
{
	for (auto it = m_owners.begin(); it != m_owners.end(); ++it)
	{
		OwnerState* st = it.value();
		if (!st || st->kind != IoSignalOwnerKind::Device)
			continue;
		const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(host.findObject(it.key().toStdString()));
		if (!device)
			continue;
		device->setIoSignalsJson(st->table.toJson());
	}
}

QJsonObject IoSignalNetwork::toProjectJson() const
{
	QJsonObject root;
	QJsonObject owners;
	for (auto it = m_owners.constBegin(); it != m_owners.constEnd(); ++it)
	{
		const OwnerState* st = it.value();
		if (!st)
			continue;
		QJsonObject o;
		o.insert(QStringLiteral("kind"),
				 st->kind == IoSignalOwnerKind::Device ? QStringLiteral("device") : QStringLiteral("robot"));
		o.insert(QStringLiteral("displayName"), st->displayName);
		o.insert(QStringLiteral("canvasX"), st->canvasX);
		o.insert(QStringLiteral("canvasY"), st->canvasY);
		if (st->kind == IoSignalOwnerKind::Robot)
			o.insert(QStringLiteral("signals"), signalsArrayFromTable(st->table));
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

bool IoSignalNetwork::fromProjectJson(const QJsonObject& root, QString* err)
{
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
			continue;
		st->canvasX = o.value(QStringLiteral("canvasX")).toDouble(st->canvasX);
		st->canvasY = o.value(QStringLiteral("canvasY")).toDouble(st->canvasY);
		if (kind == IoSignalOwnerKind::Robot && o.contains(QStringLiteral("signals")))
		{
			nlohmann::json wrap = nlohmann::json::object();
			const QByteArray raw =
				QJsonDocument(o.value(QStringLiteral("signals")).toArray()).toJson(QJsonDocument::Compact);
			wrap["signals"] = nlohmann::json::parse(raw.constData(), nullptr, false);
			if (wrap["signals"].is_discarded())
				wrap["signals"] = nlohmann::json::array();
			std::string e;
			if (!st->table.fromJson(wrap, &e))
			{
				if (err)
					*err = QString::fromStdString(e);
				return false;
			}
			resetOwnerRuntime(*st, false);
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
			w.id = makeWireId();
		loaded.push_back(w);
	}
	m_wires = loaded;
	emit networkChanged();
	return true;
}

bool IoSignalNetwork::importLegacyFlatSignals(const QJsonObject& signalsWrap, QString* err)
{
	ensureOwner(IoSignalOwnerKind::Robot, QString::fromLatin1(kLegacyOwnerId), QStringLiteral("Robot"));
	OwnerState* st = mutableOwner(QString::fromLatin1(kLegacyOwnerId));
	if (!st)
	{
		if (err)
			*err = QStringLiteral("legacy owner create failed");
		return false;
	}
	std::string e;
	const QByteArray raw = QJsonDocument(signalsWrap).toJson(QJsonDocument::Compact);
	nlohmann::json j;
	try
	{
		j = nlohmann::json::parse(raw.constData(), nullptr, true);
	}
	catch (...)
	{
		if (err)
			*err = QStringLiteral("ioSignals JSON invalid");
		return false;
	}
	if (!st->table.fromJson(j, &e))
	{
		if (err)
			*err = QString::fromStdString(e);
		return false;
	}
	resetOwnerRuntime(*st, false);
	emit networkChanged();
	return true;
}

QJsonObject IoSignalNetwork::ownerSignalsPayloadWithRuntime(const QString& ownerId) const
{
	const OwnerState* st = owner(ownerId);
	QJsonObject root;
	if (!st)
	{
		root.insert(QStringLiteral("signals"), QJsonArray());
		return root;
	}
	root.insert(QStringLiteral("signals"), signalsArrayFromTable(st->table));
	QJsonArray enriched;
	for (const QJsonValue& v : root.value(QStringLiteral("signals")).toArray())
	{
		QJsonObject o = v.toObject();
		const QString kind = o.value(QStringLiteral("kind")).toString();
		const int port = o.value(QStringLiteral("port")).toInt();
		bool forced = false;
		QString valueText = QStringLiteral("-");
		if (kind == QLatin1String("DI"))
		{
			forced = st->runtime.diForced.contains(port);
			const bool dv = forced ? st->runtime.diForced.value(port)
								   : st->runtime.di.value(port, o.value(QStringLiteral("defaultBool")).toBool());
			valueText = dv ? QStringLiteral("1") : QStringLiteral("0");
		}
		else if (kind == QLatin1String("DO"))
		{
			valueText = st->runtime.digitalOut.value(port, o.value(QStringLiteral("defaultBool")).toBool())
							? QStringLiteral("1")
							: QStringLiteral("0");
		}
		else if (kind == QLatin1String("AI"))
		{
			valueText = QString::number(st->runtime.ai.value(port, o.value(QStringLiteral("defaultAnalog")).toDouble()),
										'g', 6);
		}
		else if (kind == QLatin1String("AO"))
		{
			valueText = QString::number(st->runtime.ao.value(port, o.value(QStringLiteral("defaultAnalog")).toDouble()),
										'g', 6);
		}
		o.insert(QStringLiteral("value"), valueText);
		o.insert(QStringLiteral("forced"), forced);
		enriched.append(o);
	}
	root.insert(QStringLiteral("signals"), enriched);
	return root;
}

QJsonObject IoSignalNetwork::networkPayloadWithRuntime() const
{
	QJsonObject root;
	QJsonObject owners;
	for (auto it = m_owners.constBegin(); it != m_owners.constEnd(); ++it)
	{
		const OwnerState* st = it.value();
		if (!st)
			continue;
		QJsonObject o = ownerSignalsPayloadWithRuntime(it.key());
		o.insert(QStringLiteral("kind"),
				 st->kind == IoSignalOwnerKind::Device ? QStringLiteral("device") : QStringLiteral("robot"));
		o.insert(QStringLiteral("displayName"), st->displayName);
		o.insert(QStringLiteral("canvasX"), st->canvasX);
		o.insert(QStringLiteral("canvasY"), st->canvasY);
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
	root.insert(QStringLiteral("primaryOwnerId"), primaryRobotOwnerId());
	return root;
}

bool IoSignalNetwork::parseBoolText(const QString& t)
{
	return t == QLatin1String("1") || t.compare(QLatin1String("true"), Qt::CaseInsensitive) == 0 ||
		   t.compare(QLatin1String("on"), Qt::CaseInsensitive) == 0;
}

bool IoSignalNetwork::setRuntime(const QString& ownerId, const QString& kind, const int port, const QString& valueText,
								 const bool hasForced, const bool forced, QString* err)
{
	OwnerState* st = mutableOwner(ownerId);
	if (!st)
	{
		if (err)
			*err = QStringLiteral("unknown owner");
		return false;
	}
	if (kind == QLatin1String("DI"))
	{
		const bool v = parseBoolText(valueText);
		if (hasForced && forced)
		{
			st->runtime.diForced.insert(port, v);
			st->runtime.di.insert(port, v);
		}
		else if (hasForced && !forced)
		{
			st->runtime.diForced.remove(port);
			st->runtime.di.insert(port, v);
		}
		else
		{
			st->runtime.di.insert(port, v);
			if (st->runtime.diForced.contains(port))
				st->runtime.diForced.insert(port, v);
		}
	}
	else if (kind == QLatin1String("DO"))
	{
		st->runtime.digitalOut.insert(port, parseBoolText(valueText));
		propagateFrom(ownerId);
	}
	else if (kind == QLatin1String("AI"))
	{
		st->runtime.ai.insert(port, valueText.toDouble());
	}
	else if (kind == QLatin1String("AO"))
	{
		st->runtime.ao.insert(port, valueText.toDouble());
	}
	else
	{
		if (err)
			*err = QStringLiteral("unknown kind");
		return false;
	}
	emit ownerIoChanged(ownerId);
	return true;
}

void IoSignalNetwork::resetRuntime(const QString& ownerId, const bool keepForcedDi)
{
	if (ownerId.isEmpty())
	{
		for (auto it = m_owners.begin(); it != m_owners.end(); ++it)
		{
			if (it.value())
				resetOwnerRuntime(*it.value(), keepForcedDi);
		}
		propagateAll();
		return;
	}
	if (OwnerState* st = mutableOwner(ownerId))
	{
		resetOwnerRuntime(*st, keepForcedDi);
		propagateFrom(ownerId);
	}
}

void IoSignalNetwork::setDigitalOutput(const int port, const bool value)
{
	const QString id = primaryRobotOwnerId();
	if (id.isEmpty())
		return;
	if (OwnerState* st = mutableOwner(id))
	{
		st->runtime.digitalOut.insert(port, value);
		propagateFrom(id);
		emit ownerIoChanged(id);
	}
}

void IoSignalNetwork::setAnalogOutput(const int port, const double value)
{
	const QString id = primaryRobotOwnerId();
	if (id.isEmpty())
		return;
	if (OwnerState* st = mutableOwner(id))
	{
		st->runtime.ao.insert(port, value);
		emit ownerIoChanged(id);
	}
}

bool IoSignalNetwork::getDigitalInput(const int port, bool* outValue) const
{
	if (!outValue)
		return false;
	const QString id = primaryRobotOwnerId();
	const OwnerState* st = owner(id);
	if (!st)
	{
		*outValue = false;
		return false;
	}
	if (st->runtime.diForced.contains(port))
	{
		*outValue = st->runtime.diForced.value(port);
		return true;
	}
	*outValue = st->runtime.di.value(port, false);
	return true;
}

int IoSignalNetwork::resolveNamedPort(const std::string& signalName, const int fallbackPort) const
{
	return primaryTable().resolvePort(signalName, fallbackPort);
}

} // namespace cloudsim::host
