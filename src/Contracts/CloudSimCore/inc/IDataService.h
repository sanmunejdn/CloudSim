#pragma once

#include "CoreTypes.h"
#include "cloudsim_core_global.h"

#include <QJsonObject>

namespace cloudsim::core {

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

	virtual BBoxDto boundingBox(const ObjectId& id) const = 0;
	virtual bool hasVisualBranch(const ObjectId& id) const = 0;

	virtual QJsonObject saveObjectToJson(const ObjectId& id) const = 0;
	virtual ObjectId loadObjectFromJson(const QJsonObject& objectJson, QString* outError = nullptr) = 0;

	virtual ObjectId importFromFile(const QString& path, const ImportOptionsDto& options,
		QString* outError = nullptr) = 0;
};

} // namespace cloudsim::core
