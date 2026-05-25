#include "adapters/DataServiceAdapter.h"

#include "DocumentHost.h"
#include "DocumentHostAccess.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BackendVisualSync.h"
#include "DocumentImportFacade.h"
#include "BackendProjectObjectIo.h"
#include "OsgWidget.h"
#include "BackendRegistry.h"
#include "BackendRegistryBuiltins.h"

#include <QJsonArray>
#include <QJsonDocument>

namespace cloudsim::host {

DataServiceAdapter::DataServiceAdapter(DocumentHost& host) : m_host(host) {}

namespace {

BackendDataManager& backendOf(DocumentHost& host)
{
	return host.backend();
}

} // namespace

bool DataServiceAdapter::isValid(const core::ObjectId& id) const
{
	return !id.isEmpty() && backendOf(m_host).contains(id.toStdString());
}

void DataServiceAdapter::clear()
{
	backendOf(m_host).clear();
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
	if (!backendOf(m_host).registerData(obj))
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
	(void)outError;
	if (id.isEmpty())
	{
		return true;
	}
	const QStringList removed = m_host.removeBackendSubtree(id);
	return !removed.isEmpty() || !isValid(id);
}

core::ObjectId DataServiceAdapter::findByName(const QString& name) const
{
	const auto list = backendOf(m_host).findByName(name.toStdString());
	if (list.empty())
		return {};
	return QString::fromStdString(list.front()->id());
}

QString DataServiceAdapter::className(const core::ObjectId& id) const
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
	return obj ? QString::fromStdString(obj->className()) : QString();
}

QString DataServiceAdapter::displayName(const core::ObjectId& id) const
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
	return obj ? QString::fromStdString(obj->name()) : QString();
}

QVector<core::ObjectId> DataServiceAdapter::listChildren(const core::ObjectId& parentId) const
{
	QVector<core::ObjectId> out;
	for (const std::string& c : backendOf(m_host).childrenOf(parentId.toStdString()))
		out.append(QString::fromStdString(c));
	return out;
}

bool DataServiceAdapter::attachChild(const core::ObjectId& parentId, const core::ObjectId& childId, QString* outError)
{
	if (!backendOf(m_host).attachChild(parentId.toStdString(), childId.toStdString()))
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
	const auto obj = backendOf(m_host).getData(id.toStdString());
	if (!obj)
		return rows;
	const nlohmann::json j = obj->snapshotPropertyRows(&backendOf(m_host));
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
	const auto obj = backendOf(m_host).getData(id.toStdString());
	if (!obj)
	{
		if (outError)
			*outError = QStringLiteral("invalid object id");
		return false;
	}
	std::string err;
	if (!obj->applyPropertyChange(key.toStdString(), value.toStdString(), &err, &backendOf(m_host)))
	{
		if (outError)
			*outError = QString::fromStdString(err);
		return false;
	}
	afterDataServicePropertyChange(m_host, *obj, key);
	return true;
}

core::BBoxDto DataServiceAdapter::boundingBox(const core::ObjectId& id) const
{
	core::BBoxDto box;
	const auto obj = backendOf(m_host).getData(id.toStdString());
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
	if (id.isEmpty() || !osgWidgetFrom(m_host))
	{
		return false;
	}
	return osgWidgetFrom(m_host)->hasBackendObjectBranch(id.toStdString());
}

QJsonObject DataServiceAdapter::saveObjectToJson(const core::ObjectId& id) const
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
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
	std::shared_ptr<BackendDataBase> obj;
	if (!decodeBackendObjectFromProjectJson(objectJson, obj, outError))
	{
		return {};
	}
	// 内嵌几何走 registerAdopted，非纯 JSON 注册
	const QString sourcePath = objectJson.value(QStringLiteral("sourcePath")).toString();
	const QString sourceType = objectJson.value(QStringLiteral("sourceType")).toString();
	const QString parentId = objectJson.value(QStringLiteral("parentId")).toString();
	const QString catalogType = sourceType.isEmpty() ? QStringLiteral("Model") : sourceType;
	if (!registerAdoptedBackendObject(m_host, obj, sourcePath, catalogType, parentId, outError))
	{
		return {};
	}
	return QString::fromStdString(obj->id());
}

core::ObjectId DataServiceAdapter::importFromFile(const QString& path, const core::ImportOptionsDto& options,
	QString* outError)
{
	// 统一走导入门面
	const ImportFileKind kind = options.isPointCloud ? ImportFileKind::PointCloud : ImportFileKind::Mesh;
	const ImportFileResult imported = importFileIntoDocument(m_host, path, kind, options, outError);
	return imported.ok ? imported.rootBackendId : QString();
}

} // namespace cloudsim::host
