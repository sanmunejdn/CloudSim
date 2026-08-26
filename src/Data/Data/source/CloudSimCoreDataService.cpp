/// @file CloudSimCoreDataService.cpp
/// @brief Data 层 IDataService 实现：BackendDataManager 直驱；视觉/Follow 求解等 Host 能力降级

#include "BackendManagerDataService.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "BackendRegistry.h"
#include "BackendRegistryBuiltins.h"
#include "FollowAttachmentComponent.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "RunLogger.h"

#include "IDataService.h"

#include <QFileInfo>
#include <QJsonDocument>

#include <mutex>
#include <unordered_set>

#include <json.hpp>

namespace
{
namespace core = cloudsim::core;
using core::ObjectId;

void warnOnce(const char* key, const char* message)
{
	static std::mutex m;
	static std::unordered_set<std::string> seen;
	std::lock_guard<std::mutex> lock(m);
	if (seen.insert(key).second)
	{
		RunLogger::warn(message);
	}
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
	dto.geometryKind = geometryKindOf(obj);
	return dto;
}

class BackendManagerDataService final : public core::IDataService
{
public:
	bool isValid(const ObjectId& id) const override
	{
		return !id.isEmpty() && mgr().contains(id.toStdString());
	}

	void clear() override { mgr().clear(); }

	ObjectId registerObject(const core::RegisterObjectDto& meta, QString* outError = nullptr) override
	{
		ensureBackendBuiltinsRegistered();
		// P1-1: 先验证父存在，避免 register 成功 + attach 失败留下孤儿
		if (!meta.parentId.isEmpty() && !mgr().contains(meta.parentId.toStdString()))
		{
			if (outError)
				*outError = QStringLiteral("parentId does not exist: %1").arg(meta.parentId);
			return {};
		}
		auto obj = BackendRegistry::instance().create(meta.className.toStdString());
		if (!obj)
		{
			if (outError)
				*outError = QStringLiteral("Unknown className: %1").arg(meta.className);
			return {};
		}
		obj->setName(meta.name.toStdString());
		if (!mgr().registerData(obj))
		{
			if (outError)
				*outError = QStringLiteral("registerData failed");
			return {};
		}
		if (!meta.parentId.isEmpty())
		{
			// 防御性回滚：attachChild 在 contains 前置校验后仍可能因并发/环失败
			if (!attachChild(meta.parentId, QString::fromStdString(obj->id()), outError))
			{
				mgr().unregisterData(obj->id());
				return {};
			}
		}
		return QString::fromStdString(obj->id());
	}

	bool unregisterSubtree(const ObjectId& id, QString* outError = nullptr) override
	{
		(void)outError;
		if (id.isEmpty())
		{
			return true;
		}
		// 先子后父，父摘除后子的层级边已被清，反向序保证双方都在场
		const std::vector<std::string> ids = mgr().subtreeIds(id.toStdString());
		bool any = false;
		for (auto it = ids.rbegin(); it != ids.rend(); ++it)
		{
			any = mgr().unregisterData(*it) || any;
		}
		return any || !mgr().contains(id.toStdString());
	}

	ObjectId findByName(const QString& name) const override
	{
		const auto list = mgr().findByName(name.toStdString());
		return list.empty() ? ObjectId() : QString::fromStdString(list.front()->id());
	}

	QString className(const ObjectId& id) const override
	{
		const auto obj = mgr().getData(id.toStdString());
		return obj ? QString::fromStdString(obj->className()) : QString();
	}

	QString displayName(const ObjectId& id) const override
	{
		const auto obj = mgr().getData(id.toStdString());
		return obj ? QString::fromStdString(obj->name()) : QString();
	}

	QVector<ObjectId> listChildren(const ObjectId& parentId) const override
	{
		QVector<ObjectId> out;
		for (const std::string& c : mgr().childrenOf(parentId.toStdString()))
		{
			out.append(QString::fromStdString(c));
		}
		return out;
	}

	bool attachChild(const ObjectId& parentId, const ObjectId& childId, QString* outError = nullptr) override
	{
		if (!mgr().attachChild(parentId.toStdString(), childId.toStdString()))
		{
			if (outError)
				*outError = QStringLiteral("attachChild failed (missing id or cycle)");
			return false;
		}
		return true;
	}

	QVector<core::PropertyRowDto> propertyRows(const ObjectId& id) const override
	{
		QVector<core::PropertyRowDto> rows;
		const auto obj = mgr().getData(id.toStdString());
		if (!obj)
		{
			return rows;
		}
		const nlohmann::json j = obj->snapshotPropertyRows(&mgr());
		if (!j.is_array())
		{
			return rows;
		}
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

	// 纯数据写：Host 适配器才有的 follow 重烘焙/层级传播/视觉置脏副作用不在本层
	bool applyPropertyChange(const ObjectId& id, const QString& key, const QString& value,
							 QString* outError = nullptr) override
	{
		const auto obj = mgr().getData(id.toStdString());
		if (!obj)
		{
			if (outError)
				*outError = QStringLiteral("invalid object id");
			return false;
		}
		std::string err;
		if (!obj->applyPropertyChange(key.toStdString(), value.toStdString(), &err, &mgr()))
		{
			if (outError)
				*outError = QString::fromStdString(err);
			return false;
		}
		return true;
	}

	bool applyWorldPoseMm(const ObjectId& id, const core::PoseDto& pose, QString* outError = nullptr) override
	{
		const auto obj = mgr().getData(id.toStdString());
		if (!obj)
		{
			if (outError)
				*outError = QStringLiteral("invalid object id");
			return false;
		}
		const BackendVec3 pos{pose.positionMm.x, pose.positionMm.y, pose.positionMm.z};
		const BackendVec3 euler{pose.eulerDeg.x, pose.eulerDeg.y, pose.eulerDeg.z};
		if (backend_mat4_nearly_equal(obj->worldMatrix(), backend_world_mat_from_pose(pos, euler), 1e-5))
		{
			return true;
		}
		if (obj->supportsBackendTransform())
		{
			obj->applyBackendWorldPose(pos, euler);
		}
		else
		{
			// P2-1: 当前内建类型 supportsBackendTransform() 恒等于 hasPoseProperty()，
			// 此分支仅防御未来"hasPose=true 但 supports=false"的子类；setPose/setRotation 此时为 no-op
			obj->setPose(pos);
			obj->setRotation(euler);
		}
		return true;
	}

	bool applyColor(const ObjectId& id, const core::ColorDto& color, QString* outError = nullptr) override
	{
		const auto obj = mgr().getData(id.toStdString());
		if (!obj)
		{
			if (outError)
				*outError = QStringLiteral("invalid object id");
			return false;
		}
		BackendColor c;
		c.r = color.r;
		c.g = color.g;
		c.b = color.b;
		c.a = color.a;
		obj->setColor(c);
		return true;
	}

	bool isVisible(const ObjectId& id) const override
	{
		const auto obj = mgr().getData(id.toStdString());
		// P1-2: 非法 id 返回 false，与 isValid 语义一致（避免已删对象被误判可见）
		return obj ? obj->isVisible() : false;
	}

	bool setVisible(const ObjectId& id, bool visible, QString* outError = nullptr) override
	{
		const auto obj = mgr().getData(id.toStdString());
		if (!obj)
		{
			if (outError)
				*outError = QStringLiteral("invalid object id");
			return false;
		}
		obj->setVisible(visible);
		return true;
	}

	core::PoseDto worldPoseMm(const ObjectId& id) const override
	{
		core::PoseDto dto;
		const auto obj = mgr().getData(id.toStdString());
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

	core::BBoxDto boundingBox(const ObjectId& id) const override
	{
		core::BBoxDto box;
		const auto obj = mgr().getData(id.toStdString());
		if (!obj)
		{
			return box;
		}
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

	bool hasVisualBranch(const ObjectId& id) const override
	{
		// Data 层无视觉树，恒 false；真实值见 Host 侧 DataServiceAdapter
		(void)id;
		return false;
	}

	QJsonObject saveObjectToJson(const ObjectId& id) const override
	{
		const auto obj = mgr().getData(id.toStdString());
		if (!obj)
		{
			return {};
		}
		const nlohmann::json j = obj->saveToJson();
		const QJsonDocument doc = QJsonDocument::fromJson(QByteArray::fromStdString(j.dump()));
		return doc.isObject() ? doc.object() : QJsonObject();
	}

	ObjectId loadObjectFromJson(const QJsonObject& objectJson, QString* outError = nullptr) override
	{
		ensureBackendBuiltinsRegistered();
		const QJsonDocument doc(objectJson);
		const QByteArray bytes = doc.toJson(QJsonDocument::Compact);
		const nlohmann::json j = nlohmann::json::parse(bytes.constData(), nullptr, false);
		if (j.is_discarded() || !j.is_object())
		{
			if (outError)
				*outError = QStringLiteral("invalid object json");
			return {};
		}
		try
		{
			const std::string cls = j.value("className", "");
			auto obj = BackendRegistry::instance().create(cls);
			if (!obj)
			{
				if (outError)
					*outError = QStringLiteral("Unknown className: %1").arg(QString::fromStdString(cls));
				return {};
			}
			std::string err;
			if (!obj->loadFromJson(j, &err))
			{
				if (outError)
					*outError = QString::fromStdString(err);
				return {};
			}
			if (!mgr().registerData(obj))
			{
				if (outError)
					*outError = QStringLiteral("registerData failed (id conflict?)");
				return {};
			}
			return QString::fromStdString(obj->id());
		}
		catch (const nlohmann::json::exception& ex)
		{
			if (outError)
				*outError = QStringLiteral("json type error: %1").arg(QString::fromUtf8(ex.what()));
			return {};
		}
	}

	ObjectId importFromFile(const QString& path, const core::ImportOptionsDto& options,
							QString* outError = nullptr) override
	{
		ensureBackendBuiltinsRegistered();
		// P1-1: 先验证父存在，避免 register 成功 + attach 失败留下孤儿
		if (!options.parentId.isEmpty() && !mgr().contains(options.parentId.toStdString()))
		{
			if (outError)
				*outError = QStringLiteral("parentId does not exist: %1").arg(options.parentId);
			return {};
		}
		const std::string utf8Path = path.toStdString(); // Qt6 下为 UTF-8；与 loadFromFile 的 UTF-8 约定一致
		std::shared_ptr<BackendDataBase> obj;
		std::string err;
		if (options.isPointCloud)
		{
			auto pc = std::make_shared<PointCloudBackendData>();
			if (!pc->loadFromFile(utf8Path, &err) || !pc->hasGeometry())
			{
				if (outError)
					*outError = QString::fromStdString(err);
				return {};
			}
			obj = std::move(pc);
		}
		else
		{
			// STEP/BREP 需 GeometryAlgorithm/Host 链，本层不支持
			auto mesh = std::make_shared<MeshBackendData>();
			if (!mesh->loadFromFile(utf8Path, &err, options.meshImportQuality) || !mesh->hasGeometry())
			{
				if (outError)
					*outError = QString::fromStdString(err);
				return {};
			}
			obj = std::move(mesh);
		}
		obj->setName(QFileInfo(path).fileName().toStdString());
		if (!options.persistedId.isEmpty())
		{
			// 注册前设 id（注册即冻结）
			obj->setId(options.persistedId.toStdString());
		}
		if (!mgr().registerData(obj))
		{
			if (outError)
				*outError = QStringLiteral("registerData failed (id conflict?)");
			return {};
		}
		if (!options.parentId.isEmpty())
		{
			// 防御性回滚：attachChild 在 contains 前置校验后仍可能因并发/环失败
			if (!attachChild(options.parentId, QString::fromStdString(obj->id()), outError))
			{
				mgr().unregisterData(obj->id());
				return {};
			}
		}
		return QString::fromStdString(obj->id());
	}

	QVector<ObjectId> topoOrder() const override
	{
		QVector<ObjectId> out;
		for (const std::string& id : mgr().topoOrder())
		{
			out.append(QString::fromStdString(id));
		}
		return out;
	}

	QVector<ObjectId> listAll() const override
	{
		QVector<ObjectId> out;
		for (const auto& obj : mgr().listData())
		{
			if (obj)
			{
				out.append(QString::fromStdString(obj->id()));
			}
		}
		return out;
	}

	QVector<ObjectId> parentsOf(const ObjectId& id) const override
	{
		QVector<ObjectId> out;
		for (const std::string& pid : mgr().parentsOf(id.toStdString()))
		{
			out.append(QString::fromStdString(pid));
		}
		return out;
	}

	core::BackendObjectDto objectSnapshot(const ObjectId& id) const override
	{
		const auto obj = mgr().getData(id.toStdString());
		if (!obj)
		{
			return {};
		}
		return makeObjectSnapshot(mgr(), *obj);
	}

	QVector<core::BackendObjectDto> listObjectSnapshots() const override
	{
		QVector<core::BackendObjectDto> out;
		for (const auto& obj : mgr().listData())
		{
			if (obj)
			{
				out.append(makeObjectSnapshot(mgr(), *obj));
			}
		}
		return out;
	}

	core::GeometryKind geometryKind(const ObjectId& id) const override
	{
		const auto obj = mgr().getData(id.toStdString());
		return obj ? geometryKindOf(*obj) : core::GeometryKind::None;
	}

	bool hasComponent(const ObjectId& id, const QString& componentType) const override
	{
		const auto obj = mgr().getData(id.toStdString());
		return obj && obj->hasComponent(componentType.toStdString());
	}

	bool applyFollowTargetByName(const ObjectId& followerId, const QString& targetName,
								 QString* outError = nullptr) override
	{
		return applyPropertyChange(followerId, QStringLiteral("follow.targetName"), targetName, outError);
	}

	void markFollowDirtyFromMove(const ObjectId& seedId) override
	{
		(void)seedId;
		warnOnce("markFollowDirtyFromMove",
				 "[DataService] Data 层无 Follow 求解器/视觉同步，markFollowDirtyFromMove 空转；请经 Host 适配器调用。");
	}

	void requestFollowSolveForced() override
	{
		warnOnce("requestFollowSolveForced",
				 "[DataService] Data 层无 Follow 求解器/视觉同步，requestFollowSolveForced 空转；请经 Host 适配器调用。");
	}

	bool runFollowSolveAndSync(const core::FollowSolveContextDto& ctx, QString* outError = nullptr) override
	{
		(void)ctx;
		warnOnce("runFollowSolveAndSync",
				 "[DataService] Data 层无 Follow 求解器/视觉同步，runFollowSolveAndSync 失败；请经 Host 适配器调用。");
		if (outError)
			*outError = QStringLiteral("follow solver requires Host adapter");
		return false;
	}

	ObjectId followTargetId(const ObjectId& followerId) const override
	{
		const auto obj = mgr().getData(followerId.toStdString());
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
		const std::string tid = follow->targetBackendId();
		return tid.empty() ? ObjectId() : QString::fromStdString(tid);
	}

	QVector<ObjectId> findByClassName(const QString& className) const override
	{
		QVector<ObjectId> out;
		if (className.isEmpty())
		{
			return out;
		}
		for (const auto& obj : mgr().findByClass(className.toStdString()))
		{
			if (obj)
			{
				out.append(QString::fromStdString(obj->id()));
			}
		}
		return out;
	}

private:
	static BackendDataManager& mgr() { return BackendDataManager::instance(); }
};

} // namespace

std::unique_ptr<core::IDataService> makeBackendManagerDataService()
{
	return std::make_unique<BackendManagerDataService>();
}
