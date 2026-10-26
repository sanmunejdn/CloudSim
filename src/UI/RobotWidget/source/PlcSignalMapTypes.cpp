/// @file PlcSignalMapTypes.cpp

#include "PlcSignalMapTypes.h"

#include <QUuid>

QJsonObject externalPlcLinkToJson(const ExternalPlcLinkConfig& c)
{
	QJsonObject o;
	o.insert(QStringLiteral("protocol"), c.protocol);
	o.insert(QStringLiteral("gateway"), c.gateway);
	o.insert(QStringLiteral("port"), c.port);
	o.insert(QStringLiteral("path"), c.path);
	o.insert(QStringLiteral("cpu"), c.cpu);
	o.insert(QStringLiteral("timeoutMs"), c.timeoutMs);
	o.insert(QStringLiteral("enabled"), c.enabled);
	return o;
}

ExternalPlcLinkConfig externalPlcLinkFromJson(const QJsonObject& o)
{
	ExternalPlcLinkConfig c;
	c.protocol = o.value(QStringLiteral("protocol")).toString(QStringLiteral("modbusTcp"));
	c.gateway = o.value(QStringLiteral("gateway")).toString(QStringLiteral("127.0.0.1"));
	c.port = o.value(QStringLiteral("port")).toInt(502);
	c.path = o.value(QStringLiteral("path")).toString(QStringLiteral("1"));
	c.cpu = o.value(QStringLiteral("cpu")).toString();
	c.timeoutMs = o.value(QStringLiteral("timeoutMs")).toInt(3000);
	c.enabled = o.value(QStringLiteral("enabled")).toBool(false);
	return c;
}

QJsonArray plcSignalMapToJson(const QVector<PlcSignalMapEntry>& map)
{
	QJsonArray arr;
	for (const PlcSignalMapEntry& e : map)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), e.id);
		o.insert(QStringLiteral("plcTag"), e.plcTag);
		o.insert(QStringLiteral("ownerId"), e.ownerId);
		o.insert(QStringLiteral("signal"), e.signal);
		o.insert(QStringLiteral("direction"),
				 e.direction == PlcMapDirection::Out ? QStringLiteral("out") : QStringLiteral("in"));
		o.insert(QStringLiteral("type"), e.type.isEmpty() ? QStringLiteral("bool") : e.type);
		arr.append(o);
	}
	return arr;
}

bool plcSignalMapFromJson(const QJsonArray& arr, QVector<PlcSignalMapEntry>* out, QString* err)
{
	if (!out)
	{
		return false;
	}
	QVector<PlcSignalMapEntry> map;
	map.reserve(arr.size());
	for (const QJsonValue& v : arr)
	{
		const QJsonObject o = v.toObject();
		PlcSignalMapEntry e;
		e.id = o.value(QStringLiteral("id")).toString();
		if (e.id.isEmpty())
		{
			e.id = QUuid::createUuid().toString(QUuid::WithoutBraces);
		}
		e.plcTag = o.value(QStringLiteral("plcTag")).toString();
		e.ownerId = o.value(QStringLiteral("ownerId")).toString();
		e.signal = o.value(QStringLiteral("signal")).toString();
		const QString dir = o.value(QStringLiteral("direction")).toString(QStringLiteral("in"));
		e.direction = (dir.compare(QStringLiteral("out"), Qt::CaseInsensitive) == 0) ? PlcMapDirection::Out
																					 : PlcMapDirection::In;
		e.type = o.value(QStringLiteral("type")).toString(QStringLiteral("bool"));
		if (e.plcTag.isEmpty() || e.ownerId.isEmpty() || e.signal.isEmpty())
		{
			if (err)
			{
				*err = QStringLiteral("plcSignalMap entry missing plcTag/ownerId/signal");
			}
			return false;
		}
		map.push_back(e);
	}
	*out = map;
	return true;
}
