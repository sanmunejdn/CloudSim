#ifndef WIDGET_BACKENDUNITSTREEBINDER_H
#define WIDGET_BACKENDUNITSTREEBINDER_H

/// @file BackendUnitsTreeBinder.h
/// @brief 将 DisplayForest 单文档子树绑定到 QTreeView+Model（文档作用域）

#include "widget_global.h"

#include "BackendUnitsDisplayForest.h"

#include <QFont>
#include <QHash>
#include <QModelIndex>
#include <QPair>
#include <QString>

class QStandardItem;
class QStandardItemModel;
class QTreeView;

/// 禁止跨文档全局 takeChildren；仅 sync 指定 documentId
class WIDGET_EXPORT BackendUnitsTreeBinder
{
public:
	explicit BackendUnitsTreeBinder(QTreeView* tree);

	QStandardItemModel* itemModel() const { return m_model; }

	void setAnnotationGroupLabel(const QString& label);

	void syncDocument(const BackendUnitsDisplayDocument& doc);
	void removeDocument(const QString& documentId);
	/// 树中只保留该文档根；documentId 空则清空
	void retainOnlyDocument(const QString& documentId);
	/// 隐藏其它文档根（不销毁），用于切 Tab 复用已构建子树
	void showOnlyDocument(const QString& documentId);
	bool hasDocument(const QString& documentId) const;
	void setActiveDocument(const QString& documentId);

	QStandardItem* documentRoot(const QString& documentId) const;
	QStandardItem* annotationGroup(const QString& documentId) const;
	QStandardItem* findBackendItem(const QString& documentId, const QString& backendId) const;
	QStandardItem* findBackendItemAnyDocument(const QString& backendId) const;
	QStandardItem* itemFromIndex(const QModelIndex& index) const;

	void patchObjectVisible(const QString& documentId, const QString& backendId, bool visible);

	void addAnnotationItem(const QString& documentId, const QString& annotationId, const QString& displayText,
						   bool visible);
	void removeAnnotationItem(const QString& documentId, const QString& annotationId);
	void setAnnotationItemVisible(const QString& documentId, const QString& annotationId, bool visible);

	void setCurrentBackendItem(const QString& documentId, const QString& backendId, bool scroll);

private:
	using DocObjKey = QPair<QString, QString>;

	QStandardItem* ensureDocumentRoot(const QString& documentId, const QString& title, bool isActive);
	void clearDocumentChildrenKeepRoot(QStandardItem* docRoot, const QString& documentId);
	void applyActiveStyle(QStandardItem* docRoot, bool isActive) const;
	void forgetDocumentIndexes(const QString& documentId);
	void expandItem(QStandardItem* item) const;
	static QStandardItem* makeLabeledItem(const QString& text, int itemType, const QString& documentId,
										  bool checkable);

	QTreeView* m_tree = nullptr;
	QStandardItemModel* m_model = nullptr;
	QString m_annotationGroupLabel = QStringLiteral("Annotations");
	QHash<QString, QStandardItem*> m_documentRoots;
	QHash<QString, QStandardItem*> m_annotationGroups;
	QHash<DocObjKey, QStandardItem*> m_backendItems;
};

#endif // WIDGET_BACKENDUNITSTREEBINDER_H
