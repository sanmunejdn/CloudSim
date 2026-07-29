/// @file ProcessFlowNodeProps.cpp
/// @brief 节点属性默认值与 JSON

#include "ProcessFlowNodeProps.h"

#include <algorithm>

QJsonObject ProcessFlowNodeProps::toJson() const
{
	QJsonObject o;
	o.insert(QStringLiteral("kind"), kind);
	o.insert(QStringLiteral("cycleTimeSec"), cycleTimeSec);
	o.insert(QStringLiteral("inventoryQty"), inventoryQty);
	o.insert(QStringLiteral("capacityQty"), capacityQty);
	o.insert(QStringLiteral("setupTimeSec"), setupTimeSec);
	o.insert(QStringLiteral("priority"), priority);
	o.insert(QStringLiteral("batchSize"), batchSize);
	o.insert(QStringLiteral("scrapRate"), scrapRate);
	o.insert(QStringLiteral("mtbfSec"), mtbfSec);
	o.insert(QStringLiteral("mttrSec"), mttrSec);
	o.insert(QStringLiteral("requiredInputs"), requiredInputs);
	if (!bindingBackendId.isEmpty() || !bindingProgramId.isEmpty())
	{
		QJsonObject b;
		b.insert(QStringLiteral("backendId"), bindingBackendId);
		b.insert(QStringLiteral("programId"), bindingProgramId);
		o.insert(QStringLiteral("binding"), b);
	}
	return o;
}

ProcessFlowNodeProps ProcessFlowNodeProps::fromJson(const QJsonObject& obj)
{
	ProcessFlowNodeProps p;
	p.kind = obj.value(QStringLiteral("kind")).toString(QStringLiteral("station"));
	if (!allKinds().contains(p.kind))
	{
		p.kind = QStringLiteral("station");
	}
	p.cycleTimeSec = obj.value(QStringLiteral("cycleTimeSec")).toDouble(0.0);
	p.inventoryQty = obj.value(QStringLiteral("inventoryQty")).toDouble(0.0);
	p.capacityQty = obj.value(QStringLiteral("capacityQty")).toDouble(0.0);
	p.setupTimeSec = obj.value(QStringLiteral("setupTimeSec")).toDouble(0.0);
	p.priority = obj.value(QStringLiteral("priority")).toDouble(0.0);
	p.batchSize = obj.value(QStringLiteral("batchSize")).toDouble(1.0);
	p.scrapRate = std::clamp(obj.value(QStringLiteral("scrapRate")).toDouble(0.0), 0.0, 1.0);
	p.mtbfSec = obj.value(QStringLiteral("mtbfSec")).toDouble(0.0);
	p.mttrSec = obj.value(QStringLiteral("mttrSec")).toDouble(0.0);
	p.requiredInputs = obj.value(QStringLiteral("requiredInputs")).toDouble(2.0);
	const QJsonObject b = obj.value(QStringLiteral("binding")).toObject();
	p.bindingBackendId = b.value(QStringLiteral("backendId")).toString();
	p.bindingProgramId = b.value(QStringLiteral("programId")).toString();
	return p;
}

ProcessFlowNodeProps ProcessFlowNodeProps::defaultsForKind(const QString& kind)
{
	ProcessFlowNodeProps p;
	p.kind = allKinds().contains(kind) ? kind : QStringLiteral("station");
	p.setupTimeSec = 0.0;
	p.priority = 0.0;
	p.batchSize = 1.0;
	p.scrapRate = 0.0;
	p.mtbfSec = 0.0;
	p.mttrSec = 0.0;
	p.requiredInputs = 2.0;

	if (p.kind == QStringLiteral("start") || p.kind == QStringLiteral("end"))
	{
		p.cycleTimeSec = 0.0;
		p.inventoryQty = 0.0;
		p.capacityQty = 0.0;
	}
	else if (p.kind == QStringLiteral("buffer"))
	{
		p.cycleTimeSec = 0.0;
		p.inventoryQty = 10.0;
		p.capacityQty = 50.0;
	}
	else if (p.kind == QStringLiteral("warehouse"))
	{
		p.cycleTimeSec = 0.0;
		p.inventoryQty = 50.0;
		p.capacityQty = 500.0;
	}
	else if (p.kind == QStringLiteral("conveyor"))
	{
		p.cycleTimeSec = 10.0;
		p.inventoryQty = 0.0;
		p.capacityQty = 5.0;
	}
	else if (p.kind == QStringLiteral("agv"))
	{
		p.cycleTimeSec = 20.0;
		p.inventoryQty = 0.0;
		p.capacityQty = 2.0;
	}
	else if (p.kind == QStringLiteral("assembly"))
	{
		p.cycleTimeSec = 40.0;
		p.inventoryQty = 0.0;
		p.capacityQty = 1.0;
		p.requiredInputs = 2.0;
	}
	else if (p.kind == QStringLiteral("inspect"))
	{
		p.cycleTimeSec = 30.0;
		p.inventoryQty = 0.0;
		p.capacityQty = 1.0;
		p.scrapRate = 0.0;
	}
	else // station
	{
		p.cycleTimeSec = 30.0;
		p.inventoryQty = 0.0;
		p.capacityQty = 1.0;
	}
	return p;
}

QString ProcessFlowNodeProps::inferKindFromTitle(const QString& title, const QString& subtitle)
{
	const QString t = title + subtitle;
	if (t.contains(QStringLiteral("开始")) || t.contains(QStringLiteral("Start"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("入口")))
	{
		return QStringLiteral("start");
	}
	if (t.contains(QStringLiteral("结束")) || t.contains(QStringLiteral("End"), Qt::CaseInsensitive) ||
		t.contains(QStringLiteral("出口")))
	{
		return QStringLiteral("end");
	}
	if (t.contains(QStringLiteral("仓库")) || t.contains(QStringLiteral("Warehouse"), Qt::CaseInsensitive))
	{
		return QStringLiteral("warehouse");
	}
	if (t.contains(QStringLiteral("缓冲")) || t.contains(QStringLiteral("Buffer"), Qt::CaseInsensitive))
	{
		return QStringLiteral("buffer");
	}
	if (t.contains(QStringLiteral("输送")) || t.contains(QStringLiteral("Conveyor"), Qt::CaseInsensitive))
	{
		return QStringLiteral("conveyor");
	}
	if (t.contains(QStringLiteral("装配")) || t.contains(QStringLiteral("Assembly"), Qt::CaseInsensitive))
	{
		return QStringLiteral("assembly");
	}
	if (t.contains(QStringLiteral("检测")) || t.contains(QStringLiteral("Inspect"), Qt::CaseInsensitive))
	{
		return QStringLiteral("inspect");
	}
	if (t.contains(QStringLiteral("工位")) || t.contains(QStringLiteral("Station"), Qt::CaseInsensitive))
	{
		return QStringLiteral("station");
	}
	return QStringLiteral("station");
}

QString ProcessFlowNodeProps::displayNameZh(const QString& kind)
{
	if (kind == QStringLiteral("start"))
		return QStringLiteral("开始");
	if (kind == QStringLiteral("buffer"))
		return QStringLiteral("缓冲");
	if (kind == QStringLiteral("warehouse"))
		return QStringLiteral("仓库");
	if (kind == QStringLiteral("inspect"))
		return QStringLiteral("检测");
	if (kind == QStringLiteral("conveyor"))
		return QStringLiteral("输送");
	if (kind == QStringLiteral("agv"))
		return QStringLiteral("AGV");
	if (kind == QStringLiteral("assembly"))
		return QStringLiteral("装配");
	if (kind == QStringLiteral("end"))
		return QStringLiteral("结束");
	return QStringLiteral("工位");
}

QString ProcessFlowNodeProps::displayNameEn(const QString& kind)
{
	if (kind == QStringLiteral("start"))
		return QStringLiteral("Start");
	if (kind == QStringLiteral("buffer"))
		return QStringLiteral("Buffer");
	if (kind == QStringLiteral("warehouse"))
		return QStringLiteral("Warehouse");
	if (kind == QStringLiteral("inspect"))
		return QStringLiteral("Inspect");
	if (kind == QStringLiteral("conveyor"))
		return QStringLiteral("Conveyor");
	if (kind == QStringLiteral("agv"))
		return QStringLiteral("AGV");
	if (kind == QStringLiteral("assembly"))
		return QStringLiteral("Assembly");
	if (kind == QStringLiteral("end"))
		return QStringLiteral("End");
	return QStringLiteral("Station");
}

QStringList ProcessFlowNodeProps::allKinds()
{
	return {QStringLiteral("start"),	 QStringLiteral("station"),	  QStringLiteral("buffer"),
			QStringLiteral("warehouse"), QStringLiteral("conveyor"),  QStringLiteral("agv"),
			QStringLiteral("assembly"),	 QStringLiteral("inspect"),	  QStringLiteral("end")};
}

bool ProcessFlowNodeProps::isMachineKind(const QString& kind)
{
	return kind == QStringLiteral("station") || kind == QStringLiteral("inspect") ||
		   kind == QStringLiteral("assembly") || kind == QStringLiteral("conveyor") ||
		   kind == QStringLiteral("agv");
}

bool ProcessFlowNodeProps::isBufferKind(const QString& kind)
{
	return kind == QStringLiteral("buffer") || kind == QStringLiteral("warehouse");
}
