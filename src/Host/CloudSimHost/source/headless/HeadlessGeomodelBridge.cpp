/// @file HeadlessGeomodelBridge.cpp

#include "headless/HeadlessGeomodelBridge.h"

#include "BackendDataBase.h"
#include "BackendTypeIds.h"
#include "DocumentHost.h"
#include "ParametricBrepBackendData.h"

#include <QJsonArray>

namespace cloudsim::host
{
HeadlessGeomodelBridge::HeadlessGeomodelBridge(DocumentHost& host) : m_host(host) {}

QJsonObject HeadlessGeomodelBridge::summaryJson() const
{
	QJsonArray bodies;
	for (const std::shared_ptr<BackendDataBase>& obj : m_host.listObjects())
	{
		if (!obj || obj->className() != backend_type::kClassParametricBrep)
			continue;
		const auto* param = dynamic_cast<const ParametricBrepBackendData*>(obj.get());
		QJsonObject entry;
		entry.insert(QStringLiteral("backendId"), QString::fromStdString(obj->id()));
		entry.insert(QStringLiteral("name"), QString::fromStdString(obj->name()));
		entry.insert(QStringLiteral("className"), QString::fromStdString(obj->className()));
		entry.insert(QStringLiteral("hasGeometry"), obj->hasGeometry());
		if (param)
			entry.insert(QStringLiteral("featureCount"), static_cast<int>(param->features().size()));
		bodies.append(entry);
	}

	QJsonObject o;
	o.insert(QStringLiteral("ok"), true);
	o.insert(QStringLiteral("bodies"), bodies);
	o.insert(QStringLiteral("count"), bodies.size());
	return o;
}

} // namespace cloudsim::host
