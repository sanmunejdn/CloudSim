#ifndef CLOUDSIMCORE_IDATASERVICE_H
#define CLOUDSIMCORE_IDATASERVICE_H

/// @file IDataService.h
/// @brief 文档后端数据服务

#include "cloudsim_core_global.h"

#include "CoreTypes.h"

#include <QJsonObject>

namespace cloudsim::core
{
/// 文档后端数据服务
class CLOUDSIM_CORE_EXPORT IDataService
{
public:
	virtual ~IDataService() = default;

	virtual bool isValid(const ObjectId& id) const = 0;
	virtual void clear() = 0;

	virtual ObjectId registerObject(const RegisterObjectDto& meta, QString* outError = nullptr) = 0;
	virtual bool unregisterSubtree(const ObjectId& id, QString* outError = nullptr) = 0;

	virtual ObjectId findByName(const QString& name) const = 0;
	virtual QString className(const ObjectId& id) const = 0;
	virtual QString displayName(const ObjectId& id) const = 0;
	virtual QVector<ObjectId> listChildren(const ObjectId& parentId) const = 0;
	virtual bool attachChild(const ObjectId& parentId, const ObjectId& childId, QString* outError = nullptr) = 0;

	virtual QVector<PropertyRowDto> propertyRows(const ObjectId& id) const = 0;
	virtual bool applyPropertyChange(const ObjectId& id, const QString& key, const QString& value,
									 QString* outError = nullptr) = 0;

	/// OSG gizmo 写回世界位姿（pose 相对几何中心 + 欧拉角）
	virtual bool applyWorldPoseMm(const ObjectId& id, const PoseDto& pose, QString* outError = nullptr) = 0;
	virtual bool applyColor(const ObjectId& id, const ColorDto& color, QString* outError = nullptr) = 0;
	virtual bool isVisible(const ObjectId& id) const = 0;
	virtual bool setVisible(const ObjectId& id, bool visible, QString* outError = nullptr) = 0;
	virtual PoseDto worldPoseMm(const ObjectId& id) const = 0;

	virtual BBoxDto boundingBox(const ObjectId& id) const = 0;
	virtual bool hasVisualBranch(const ObjectId& id) const = 0;

	virtual QJsonObject saveObjectToJson(const ObjectId& id) const = 0;
	virtual ObjectId loadObjectFromJson(const QJsonObject& objectJson, QString* outError = nullptr) = 0;

	virtual ObjectId importFromFile(const QString& path, const ImportOptionsDto& options,
									QString* outError = nullptr) = 0;

	// 树构建支持
	virtual QVector<ObjectId> topoOrder() const = 0;
	virtual QVector<ObjectId> listAll() const = 0;
	virtual QVector<ObjectId> parentsOf(const ObjectId& id) const = 0;

	virtual BackendObjectDto objectSnapshot(const ObjectId& id) const = 0;
	virtual QVector<BackendObjectDto> listObjectSnapshots() const = 0;
	virtual GeometryKind geometryKind(const ObjectId& id) const = 0;
	virtual bool hasComponent(const ObjectId& id, const QString& componentType) const = 0;

	virtual bool applyFollowTargetByName(const ObjectId& followerId, const QString& targetName,
										 QString* outError = nullptr) = 0;
	virtual void markFollowDirtyFromMove(const ObjectId& seedId) = 0;
	virtual void requestFollowSolveForced() = 0;
	virtual bool runFollowSolveAndSync(const FollowSolveContextDto& ctx, QString* outError = nullptr) = 0;
};

} // namespace cloudsim::core

#endif // CLOUDSIMCORE_IDATASERVICE_H
