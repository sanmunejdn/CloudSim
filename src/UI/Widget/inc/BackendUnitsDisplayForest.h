#ifndef WIDGET_BACKENDUNITSDISPLAYFOREST_H
#define WIDGET_BACKENDUNITSDISPLAYFOREST_H

/// @file BackendUnitsDisplayForest.h
/// @brief Units 显示森林：主父投影，无 Qt

#include "widget_global.h"

#include "CoreTypes.h"

#include <QHash>
#include <QString>
#include <QVector>

/// 显示侧对象节点（1 对象 1 节点）
struct WIDGET_EXPORT BackendUnitsDisplayObject
{
	QString id;
	QString name;
	QString className;
	/// 空表示挂文档根
	QString primaryParentId;
	bool visible = true;
};

struct WIDGET_EXPORT BackendUnitsDisplayAnnotation
{
	QString id;
	QString displayText;
	bool visible = true;
};

struct WIDGET_EXPORT BackendUnitsDisplayDocument
{
	QString documentId;
	QString title;
	bool isActive = false;
	QVector<BackendUnitsDisplayAnnotation> annotations;
	QHash<QString, BackendUnitsDisplayObject> objects;
	/// 创建顺序：先全部对象，再按主父挂接时用
	QVector<QString> objectOrder;
};

/// 从 IDataService 快照构建单文档显示树（主父边；丢弃次父）
class WIDGET_EXPORT BackendUnitsDisplayForest
{
public:
	static BackendUnitsDisplayDocument buildDocument(const QString& documentId, const QString& title, bool isActive,
													 const QVector<cloudsim::core::BackendObjectDto>& snapshots,
													 const QVector<cloudsim::core::AnnotationSnapshotDto>& annotations);
};

#endif // WIDGET_BACKENDUNITSDISPLAYFOREST_H
