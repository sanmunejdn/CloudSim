/// @file DataServiceAdapter.cpp
/// @brief DataServiceAdapter 实现

#include "adapters/DataServiceAdapter.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BackendFollowSolve.h"
#include "BackendProjectObjectIo.h"
#include "BackendRegistry.h"
#include "BackendRegistryBuiltins.h"
#include "BackendVisualSync.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "DocumentImportFacade.h"
#include "FollowAttachmentComponent.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"

#include <QJsonArray>
#include <QJsonDocument>

namespace cloudsim::host
{
DataServiceAdapter::DataServiceAdapter(DocumentHost& host) : m_host(host) {}

namespace
{
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
	if (key.startsWith(QStringLiteral("follow.")))
	{
		afterFollowPropertyEdited(m_host, id, key, value);
	}
	return true;
}

bool DataServiceAdapter::applyWorldPoseMm(const core::ObjectId& id, const core::PoseDto& pose, QString* outError)
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
	if (!obj)
	{
		if (outError)
		{
			*outError = QStringLiteral("invalid object id");
		}
		return false;
	}
	BackendVec3 pos{pose.positionMm.x, pose.positionMm.y, pose.positionMm.z};
	BackendVec3 euler{pose.eulerDeg.x, pose.eulerDeg.y, pose.eulerDeg.z};
	if (obj->supportsBackendTransform())
	{
		obj->applyBackendWorldPose(pos, euler);
	}
	else
	{
		obj->setPose(pos);
		obj->setRotation(euler);
	}
	afterDataServicePropertyChange(m_host, *obj, QStringLiteral("pose.x"));
	return true;
}

bool DataServiceAdapter::applyColor(const core::ObjectId& id, const core::ColorDto& color, QString* outError)
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
	if (!obj)
	{
		if (outError)
		{
			*outError = QStringLiteral("invalid object id");
		}
		return false;
	}
	BackendColor c;
	c.r = color.r;
	c.g = color.g;
	c.b = color.b;
	c.a = color.a;
	obj->setColor(c);
	afterDataServicePropertyChange(m_host, *obj, QStringLiteral("color.r"));
	return true;
}

bool DataServiceAdapter::isVisible(const core::ObjectId& id) const
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
	return obj ? obj->isVisible() : true;
}

bool DataServiceAdapter::setVisible(const core::ObjectId& id, bool visible, QString* outError)
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
	if (!obj)
	{
		if (outError)
		{
			*outError = QStringLiteral("invalid object id");
		}
		return false;
	}
	obj->setVisible(visible);
	return true;
}

core::PoseDto DataServiceAdapter::worldPoseMm(const core::ObjectId& id) const
{
	core::PoseDto dto;
	const auto obj = backendOf(m_host).getData(id.toStdString());
	if (!obj)
	{
		return dto;
	}
	const BackendVec3 pos = obj->pose();
	const BackendVec3 rot = obj->rotation();
	dto.positionMm.x = pos.x;
	dto.positionMm.y = pos.y;
	dto.positionMm.z = pos.z;
	dto.eulerDeg.x = rot.x;
	dto.eulerDeg.y = rot.y;
	dto.eulerDeg.z = rot.z;
	return dto;
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

QVector<core::ObjectId> DataServiceAdapter::topoOrder() const
{
	QVector<core::ObjectId> out;
	for (const std::string& id : backendOf(m_host).topoOrder())
		out.append(QString::fromStdString(id));
	return out;
}

QVector<core::ObjectId> DataServiceAdapter::listAll() const
{
	QVector<core::ObjectId> out;
	for (const auto& obj : backendOf(m_host).listData())
		out.append(QString::fromStdString(obj->id()));
	return out;
}

QVector<core::ObjectId> DataServiceAdapter::parentsOf(const core::ObjectId& id) const
{
	QVector<core::ObjectId> out;
	for (const std::string& pid : backendOf(m_host).parentsOf(id.toStdString()))
		out.append(QString::fromStdString(pid));
	return out;
}

namespace
{
core::BackendObjectDto makeObjectSnapshot(const BackendDataManager& mgr, const BackendDataBase& obj)
{
	core::BackendObjectDto dto;
	dto.id = QString::fromStdString(obj.id());
	dto.name = QString::fromStdString(obj.name());
	dto.className = QString::fromStdString(obj.className());
	dto.hasGeometry = obj.hasGeometry();
	dto.visible = obj.isVisible();
	for (const std::string& pid : mgr.parentsOf(obj.id()))
	{
		dto.parentIds.append(QString::fromStdString(pid));
	}
	for (const std::string& cid : mgr.childrenOf(obj.id()))
	{
		dto.childIds.append(QString::fromStdString(cid));
	}
	const BackendBoundingBox bb = obj.geometryBounds();
	dto.bbox.valid = bb.valid;
	dto.bbox.min.x = bb.min.x;
	dto.bbox.min.y = bb.min.y;
	dto.bbox.min.z = bb.min.z;
	dto.bbox.max.x = bb.max.x;
	dto.bbox.max.y = bb.max.y;
	dto.bbox.max.z = bb.max.z;
	if (dynamic_cast<const PointCloudBackendData*>(&obj))
	{
		dto.geometryKind = core::GeometryKind::Points;
	}
	else if (dynamic_cast<const MeshBackendData*>(&obj))
	{
		dto.geometryKind = core::GeometryKind::Mesh;
	}
	else
	{
		dto.geometryKind = dto.hasGeometry ? core::GeometryKind::Mesh : core::GeometryKind::None;
	}
	return dto;
}

core::GeometryKind geometryKindOf(const BackendDataBase& obj)
{
	if (dynamic_cast<const PointCloudBackendData*>(&obj))
	{
		return core::GeometryKind::Points;
	}
	if (dynamic_cast<const MeshBackendData*>(&obj))
	{
		return core::GeometryKind::Mesh;
	}
	return obj.hasGeometry() ? core::GeometryKind::Mesh : core::GeometryKind::None;
}

} // namespace

core::BackendObjectDto DataServiceAdapter::objectSnapshot(const core::ObjectId& id) const
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
	if (!obj)
	{
		return {};
	}
	return makeObjectSnapshot(backendOf(m_host), *obj);
}

QVector<core::BackendObjectDto> DataServiceAdapter::listObjectSnapshots() const
{
	QVector<core::BackendObjectDto> out;
	const BackendDataManager& mgr = backendOf(m_host);
	for (const auto& obj : mgr.listData())
	{
		if (obj)
		{
			out.append(makeObjectSnapshot(mgr, *obj));
		}
	}
	return out;
}

core::GeometryKind DataServiceAdapter::geometryKind(const core::ObjectId& id) const
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
	if (!obj)
	{
		return core::GeometryKind::None;
	}
	return geometryKindOf(*obj);
}

bool DataServiceAdapter::hasComponent(const core::ObjectId& id, const QString& componentType) const
{
	const auto obj = backendOf(m_host).getData(id.toStdString());
	if (!obj)
	{
		return false;
	}
	return obj->hasComponent(componentType.toStdString());
}

bool DataServiceAdapter::applyFollowTargetByName(const core::ObjectId& followerId, const QString& targetName,
												 QString* outError)
{
	return applyPropertyChange(followerId, QStringLiteral("follow.targetName"), targetName, outError);
}

void DataServiceAdapter::markFollowDirtyFromMove(const core::ObjectId& seedId)
{
	if (!seedId.isEmpty())
	{
		m_host.markFollowAttachmentDirtyFromBackendMove(seedId.toStdString());
	}
}

void DataServiceAdapter::requestFollowSolveForced()
{
	m_host.requestFollowSolveForced();
}

bool DataServiceAdapter::runFollowSolveAndSync(const core::FollowSolveContextDto& ctx, QString* outError)
{
	(void)outError;
	OsgWidget* osg = osgWidgetFrom(m_host);
	if (!osg)
	{
		return false;
	}
	FollowSolveContext hostCtx;
	hostCtx.skipAll = [ctx]() { return ctx.skipAll; };
	hostCtx.fillGizmoSelectedId = [ctx](std::string& outSelectedId) -> bool
	{
		if (ctx.gizmoSelectedBackendId.isEmpty())
		{
			return false;
		}
		outSelectedId = ctx.gizmoSelectedBackendId.toStdString();
		return true;
	};
	const std::string manualStd = ctx.manualPoseAuthorityBackendId.toStdString();
	const std::string* manualPtr = ctx.manualPoseAuthorityBackendId.isEmpty() ? nullptr : &manualStd;
	runBackendFollowSolveAndSync(m_host, *osg, &hostCtx, manualPtr);
	return true;
}

core::ObjectId DataServiceAdapter::followTargetId(const core::ObjectId& followerId) const
{
	const auto obj = backendOf(m_host).getData(followerId.toStdString());
	if (!obj)
	{
		return {};
	}
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		obj->getComponent(FollowAttachmentComponent::typeKeyStatic()));
	if (!follow || !follow->enabled())
	{
		return {};
	}
	const std::string& tid = follow->targetBackendId();
	if (tid.empty())
	{
		return {};
	}
	return QString::fromStdString(tid);
}

} // namespace cloudsim::host
