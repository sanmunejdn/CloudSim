#include "adapters/DataServiceAdapter.h"

#include "BackendDataManager.h"
#include "BackendDataBase.h"
#include "BackendRegistry.h"
#include "BackendRegistryBuiltins.h"

#include <QJsonArray>
#include <QJsonDocument>

namespace cloudsim::host {

DataServiceAdapter::DataServiceAdapter(BackendDataManager& backend) : m_backend(backend) {}

bool DataServiceAdapter::isValid(const core::ObjectId& id) const
{
	return !id.isEmpty() && m_backend.contains(id.toStdString());
}

void DataServiceAdapter::clear()
{
	m_backend.clear();
}

core::ObjectId DataServiceAdapter::registerObject(const core::RegisterObjectDto& meta, QString* outError)
{
	ensureBackendBuiltinsRegistered();
	auto obj = BackendRegistry::instance().create(meta.className.toStdString());
	if (!obj)
	{
		if (outError)
			*outError = QStringLiteral("Unknown className: %1").arg(meta.className);
		return {};
	}
	obj->setName(meta.name.toStdString());
	if (!m_backend.registerData(obj))
	{
		if (outError)
			*outError = QStringLiteral("registerData failed");
		return {};
	}
	if (!meta.parentId.isEmpty())
		attachChild(meta.parentId, QString::fromStdString(obj->id()), outError);
	return QString::fromStdString(obj->id());
}

bool DataServiceAdapter::unregisterSubtree(const core::ObjectId& id, QString* outError)
{
	if (!isValid(id))
		return true;
	const std::vector<std::string> subtree = m_backend.descendantsOf(id.toStdString());
	for (const std::string& cid : subtree)
		m_backend.unregisterData(cid);
	m_backend.unregisterData(id.toStdString());
	return true;
}

core::ObjectId DataServiceAdapter::findByName(const QString& name) const
{
	const auto list = m_backend.findByName(name.toStdString());
	if (list.empty())
		return {};
	return QString::fromStdString(list.front()->id());
}

QString DataServiceAdapter::className(const core::ObjectId& id) const
{
	const auto obj = m_backend.getData(id.toStdString());
	return obj ? QString::fromStdString(obj->className()) : QString();
}

QString DataServiceAdapter::displayName(const core::ObjectId& id) const
{
	const auto obj = m_backend.getData(id.toStdString());
	return obj ? QString::fromStdString(obj->name()) : QString();
}

QVector<core::ObjectId> DataServiceAdapter::listChildren(const core::ObjectId& parentId) const
{
	QVector<core::ObjectId> out;
	for (const std::string& c : m_backend.childrenOf(parentId.toStdString()))
		out.append(QString::fromStdString(c));
	return out;
}

bool DataServiceAdapter::attachChild(const core::ObjectId& parentId, const core::ObjectId& childId, QString* outError)
{
	if (!m_backend.attachChild(parentId.toStdString(), childId.toStdString()))
	{
		if (outError)
			*outError = QStringLiteral("attachChild failed");
		return false;
	}
	return true;
}

QVector<core::PropertyRowDto> DataServiceAdapter::propertyRows(const core::ObjectId& id) const
{
	QVector<core::PropertyRowDto> rows;
	const auto obj = m_backend.getData(id.toStdString());
	if (!obj)
		return rows;
	const nlohmann::json j = obj->snapshotPropertyRows(&m_backend);
	if (!j.is_array())
		return rows;
	for (const auto& row : j)
	{
		core::PropertyRowDto dto;
		dto.key = QString::fromStdString(row.value("key", ""));
		dto.labelEn = QString::fromStdString(row.value("labelEn", ""));
		dto.editable = row.value("editable", true);
		dto.value = QString::fromStdString(row.value("value", ""));
		rows.append(dto);
	}
	return rows;
}

bool DataServiceAdapter::applyPropertyChange(const core::ObjectId& id, const QString& key, const QString& value,
	QString* outError)
{
	const auto obj = m_backend.getData(id.toStdString());
	if (!obj)
	{
		if (outError)
			*outError = QStringLiteral("invalid object id");
		return false;
	}
	std::string err;
	if (!obj->applyPropertyChange(key.toStdString(), value.toStdString(), &err, &m_backend))
	{
		if (outError)
			*outError = QString::fromStdString(err);
		return false;
	}
	return true;
}

core::BBoxDto DataServiceAdapter::boundingBox(const core::ObjectId& id) const
{
	core::BBoxDto box;
	const auto obj = m_backend.getData(id.toStdString());
	if (!obj)
		return box;
	const BackendBoundingBox bb = obj->geometryBounds();
	box.valid = bb.valid;
	box.min.x = bb.min.x;
	box.min.y = bb.min.y;
	box.min.z = bb.min.z;
	box.max.x = bb.max.x;
	box.max.y = bb.max.y;
	box.max.z = bb.max.z;
	return box;
}

bool DataServiceAdapter::hasVisualBranch(const core::ObjectId& id) const
{
	(void)id;
	return false;
}

QJsonObject DataServiceAdapter::saveObjectToJson(const core::ObjectId& id) const
{
	const auto obj = m_backend.getData(id.toStdString());
	if (!obj)
		return {};
	const nlohmann::json j = obj->saveToJson();
	const std::string dumped = j.dump();
	const QByteArray bytes = QByteArray::fromStdString(dumped);
	const QJsonDocument doc = QJsonDocument::fromJson(bytes);
	return doc.isObject() ? doc.object() : QJsonObject();
}

core::ObjectId DataServiceAdapter::loadObjectFromJson(const QJsonObject& objectJson, QString* outError)
{
	ensureBackendBuiltinsRegistered();
	const QString className = objectJson.value(QStringLiteral("className")).toString();
	auto obj = BackendRegistry::instance().create(className.toStdString());
	if (!obj)
	{
		if (outError)
			*outError = QStringLiteral("create failed for className");
		return {};
	}
	const QJsonDocument doc(objectJson);
	const std::string jsonStd = doc.toJson(QJsonDocument::Compact).toStdString();
	nlohmann::json j = nlohmann::json::parse(jsonStd, nullptr, false);
	if (j.is_discarded() || !obj->loadFromJson(j))
	{
		if (outError)
			*outError = QStringLiteral("loadFromJson failed");
		return {};
	}
	if (!m_backend.registerData(obj))
	{
		if (outError)
			*outError = QStringLiteral("registerData failed");
		return {};
	}
	return QString::fromStdString(obj->id());
}

core::ObjectId DataServiceAdapter::importFromFile(const QString& path, const core::ImportOptionsDto& options,
	QString* outError)
{
	(void)path;
	(void)options;
	if (outError)
		*outError = QStringLiteral("importFromFile: use Widget registerBackendObject path");
	return {};
}

} // namespace cloudsim::host
