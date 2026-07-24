/// @file BackendUnitsDisplayForest.cpp
/// @brief Units 显示森林实现

#include "BackendUnitsDisplayForest.h"

BackendUnitsDisplayDocument BackendUnitsDisplayForest::buildDocument(
	const QString& documentId, const QString& title, bool isActive,
	const QVector<cloudsim::core::BackendObjectDto>& snapshots,
	const QVector<cloudsim::core::AnnotationSnapshotDto>& annotations)
{
	BackendUnitsDisplayDocument doc;
	doc.documentId = documentId;
	doc.title = title;
	doc.isActive = isActive;
	doc.objectOrder.reserve(snapshots.size());
	doc.objects.reserve(snapshots.size());

	QHash<QString, bool> idSet;
	idSet.reserve(snapshots.size());
	for (const cloudsim::core::BackendObjectDto& dto : snapshots)
	{
		if (dto.id.isEmpty())
		{
			continue;
		}
		idSet.insert(dto.id, true);
	}

	for (const cloudsim::core::BackendObjectDto& dto : snapshots)
	{
		if (dto.id.isEmpty())
		{
			continue;
		}
		BackendUnitsDisplayObject obj;
		obj.id = dto.id;
		obj.name = dto.name;
		obj.className = dto.className;
		obj.visible = dto.visible;
		// 主父投影：缺父或父不在本快照则挂文档根
		if (!dto.parentIds.isEmpty())
		{
			const QString primary = dto.parentIds.front();
			if (!primary.isEmpty() && idSet.contains(primary) && primary != dto.id)
			{
				obj.primaryParentId = primary;
			}
		}
		doc.objects.insert(obj.id, obj);
		doc.objectOrder.append(obj.id);
	}

	doc.annotations.reserve(annotations.size());
	for (const cloudsim::core::AnnotationSnapshotDto& a : annotations)
	{
		BackendUnitsDisplayAnnotation ann;
		ann.id = a.id;
		ann.displayText = a.displayText.isEmpty() ? a.id : a.displayText;
		ann.visible = a.visible;
		doc.annotations.append(ann);
	}
	return doc;
}
