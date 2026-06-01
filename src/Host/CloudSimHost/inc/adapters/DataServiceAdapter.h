#pragma once

#include "IDataService.h"

namespace cloudsim::host {

class DocumentHost;

/// Backend 数据适配
class DataServiceAdapter final : public core::IDataService
{
public:
	explicit DataServiceAdapter(DocumentHost& host);

	bool isValid(const core::ObjectId& id) const override;
	void clear() override;

	core::ObjectId registerObject(const core::RegisterObjectDto& meta, QString* outError = nullptr) override;
	bool unregisterSubtree(const core::ObjectId& id, QString* outError = nullptr) override;

	core::ObjectId findByName(const QString& name) const override;
	QString className(const core::ObjectId& id) const override;
	QString displayName(const core::ObjectId& id) const override;
	QVector<core::ObjectId> listChildren(const core::ObjectId& parentId) const override;
	bool attachChild(const core::ObjectId& parentId, const core::ObjectId& childId, QString* outError = nullptr) override;

	QVector<core::PropertyRowDto> propertyRows(const core::ObjectId& id) const override;
	bool applyPropertyChange(const core::ObjectId& id, const QString& key, const QString& value,
		QString* outError = nullptr) override;

	core::BBoxDto boundingBox(const core::ObjectId& id) const override;
	bool hasVisualBranch(const core::ObjectId& id) const override;

	QJsonObject saveObjectToJson(const core::ObjectId& id) const override;
	core::ObjectId loadObjectFromJson(const QJsonObject& objectJson, QString* outError = nullptr) override;

	core::ObjectId importFromFile(const QString& path, const core::ImportOptionsDto& options,
		QString* outError = nullptr) override;

	QVector<core::ObjectId> topoOrder() const override;
	QVector<core::ObjectId> listAll() const override;
	QVector<core::ObjectId> parentsOf(const core::ObjectId& id) const override;

private:
	DocumentHost& m_host;
};

} // namespace cloudsim::host
