/// @file BackendUnitsTreeBinder.cpp
/// @brief Units 树文档作用域绑定（QTreeView 便于长列表滚动复用 viewport）

#include "BackendUnitsTreeBinder.h"

#include "MainWindow_p.h"

#include <QSignalBlocker>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTreeView>

#include <QModelIndex>

using namespace mainwindow_detail;

BackendUnitsTreeBinder::BackendUnitsTreeBinder(QTreeView* tree) : m_tree(tree)
{
	if (!m_tree)
	{
		return;
	}
	m_model = new QStandardItemModel(m_tree);
	m_tree->setModel(m_model);
	m_tree->setHeaderHidden(true);
}

QStandardItem* BackendUnitsTreeBinder::makeLabeledItem(const QString& text, int itemType, const QString& documentId,
													   bool checkable)
{
	auto* item = new QStandardItem(text);
	item->setEditable(false);
	item->setData(itemType, kRoleItemType);
	item->setData(documentId, kRoleDocumentId);
	Qt::ItemFlags flags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
	if (checkable)
	{
		flags |= Qt::ItemIsUserCheckable;
	}
	item->setFlags(flags);
	return item;
}

void BackendUnitsTreeBinder::expandItem(QStandardItem* item) const
{
	if (!m_tree || !item)
	{
		return;
	}
	m_tree->expand(item->index());
}

void BackendUnitsTreeBinder::setAnnotationGroupLabel(const QString& label)
{
	m_annotationGroupLabel = label;
	for (auto it = m_annotationGroups.begin(); it != m_annotationGroups.end(); ++it)
	{
		if (it.value())
		{
			it.value()->setText(m_annotationGroupLabel);
		}
	}
}

QStandardItem* BackendUnitsTreeBinder::documentRoot(const QString& documentId) const
{
	return m_documentRoots.value(documentId, nullptr);
}

QStandardItem* BackendUnitsTreeBinder::annotationGroup(const QString& documentId) const
{
	return m_annotationGroups.value(documentId, nullptr);
}

QStandardItem* BackendUnitsTreeBinder::findBackendItem(const QString& documentId, const QString& backendId) const
{
	return m_backendItems.value(DocObjKey(documentId, backendId), nullptr);
}

QStandardItem* BackendUnitsTreeBinder::findBackendItemAnyDocument(const QString& backendId) const
{
	for (auto it = m_backendItems.constBegin(); it != m_backendItems.constEnd(); ++it)
	{
		if (it.key().second == backendId)
		{
			return it.value();
		}
	}
	return nullptr;
}

QStandardItem* BackendUnitsTreeBinder::itemFromIndex(const QModelIndex& index) const
{
	return m_model ? m_model->itemFromIndex(index) : nullptr;
}

void BackendUnitsTreeBinder::applyActiveStyle(QStandardItem* docRoot, bool isActive) const
{
	if (!docRoot || !m_model)
	{
		return;
	}
	QFont font = docRoot->font();
	const bool wantBold = isActive;
	if (font.bold() == wantBold)
	{
		return;
	}
	font.setBold(wantBold);
	const QSignalBlocker guard(m_model);
	docRoot->setFont(font);
}

void BackendUnitsTreeBinder::setActiveDocument(const QString& documentId)
{
	for (auto it = m_documentRoots.begin(); it != m_documentRoots.end(); ++it)
	{
		applyActiveStyle(it.value(), !documentId.isEmpty() && it.key() == documentId);
	}
}

void BackendUnitsTreeBinder::forgetDocumentIndexes(const QString& documentId)
{
	QList<DocObjKey> keys;
	for (auto it = m_backendItems.constBegin(); it != m_backendItems.constEnd(); ++it)
	{
		if (it.key().first == documentId)
		{
			keys.append(it.key());
		}
	}
	for (const DocObjKey& k : keys)
	{
		m_backendItems.remove(k);
	}
	m_annotationGroups.remove(documentId);
}

void BackendUnitsTreeBinder::clearDocumentChildrenKeepRoot(QStandardItem* docRoot, const QString& documentId)
{
	if (!docRoot)
	{
		return;
	}
	forgetDocumentIndexes(documentId);
	docRoot->removeRows(0, docRoot->rowCount());
}

QStandardItem* BackendUnitsTreeBinder::ensureDocumentRoot(const QString& documentId, const QString& title,
														  bool isActive)
{
	if (!m_model || documentId.isEmpty())
	{
		return nullptr;
	}
	QStandardItem* root = m_documentRoots.value(documentId, nullptr);
	if (!root)
	{
		root = makeLabeledItem(title, kItemTypeDocument, documentId, false);
		m_model->appendRow(root);
		m_documentRoots.insert(documentId, root);
	}
	else
	{
		root->setText(title);
	}
	applyActiveStyle(root, isActive);
	expandItem(root);
	return root;
}

void BackendUnitsTreeBinder::syncDocument(const BackendUnitsDisplayDocument& doc)
{
	if (!m_model || doc.documentId.isEmpty())
	{
		return;
	}
	const QSignalBlocker guard(m_model);
	QStandardItem* docRoot = ensureDocumentRoot(doc.documentId, doc.title, doc.isActive);
	if (!docRoot)
	{
		return;
	}
	clearDocumentChildrenKeepRoot(docRoot, doc.documentId);

	QStandardItem* annGroup = makeLabeledItem(m_annotationGroupLabel, kItemTypeAnnotationGroup, doc.documentId, false);
	docRoot->appendRow(annGroup);
	m_annotationGroups.insert(doc.documentId, annGroup);

	for (const BackendUnitsDisplayAnnotation& a : doc.annotations)
	{
		if (a.id.isEmpty())
		{
			continue;
		}
		QStandardItem* item = makeLabeledItem(a.displayText, kItemTypeAnnotation, doc.documentId, true);
		item->setData(a.id, kRoleAnnotationId);
		item->setCheckState(a.visible ? Qt::Checked : Qt::Unchecked);
		annGroup->appendRow(item);
	}

	QHash<QString, QStandardItem*> idToItem;
	idToItem.reserve(doc.objectOrder.size());
	for (const QString& id : doc.objectOrder)
	{
		const BackendUnitsDisplayObject obj = doc.objects.value(id);
		if (obj.id.isEmpty())
		{
			continue;
		}
		const QString nodeText = QStringLiteral("%1 [%2]").arg(obj.name).arg(obj.id);
		QStandardItem* item = makeLabeledItem(nodeText, kItemTypeBackend, doc.documentId, true);
		item->setData(obj.id, kRoleBackendId);
		item->setCheckState(obj.visible ? Qt::Checked : Qt::Unchecked);
		idToItem.insert(obj.id, item);
		m_backendItems.insert(DocObjKey(doc.documentId, obj.id), item);
	}

	for (const QString& id : doc.objectOrder)
	{
		const BackendUnitsDisplayObject obj = doc.objects.value(id);
		QStandardItem* item = idToItem.value(id, nullptr);
		if (!item)
		{
			continue;
		}
		QStandardItem* parentItem = docRoot;
		if (!obj.primaryParentId.isEmpty())
		{
			if (QStandardItem* p = idToItem.value(obj.primaryParentId, nullptr))
			{
				parentItem = p;
			}
		}
		parentItem->appendRow(item);
	}

	expandItem(annGroup);
	expandItem(docRoot);
}

void BackendUnitsTreeBinder::removeDocument(const QString& documentId)
{
	if (!m_model || documentId.isEmpty())
	{
		return;
	}
	QStandardItem* root = m_documentRoots.take(documentId);
	forgetDocumentIndexes(documentId);
	if (!root)
	{
		return;
	}
	const int row = root->row();
	if (row >= 0)
	{
		m_model->removeRow(row);
	}
	else
	{
		delete root;
	}
}

void BackendUnitsTreeBinder::retainOnlyDocument(const QString& documentId)
{
	if (!m_model)
	{
		return;
	}
	const QSignalBlocker guard(m_model);
	const QStringList ids = m_documentRoots.keys();
	for (const QString& id : ids)
	{
		if (id != documentId)
		{
			removeDocument(id);
		}
	}
}

void BackendUnitsTreeBinder::showOnlyDocument(const QString& documentId)
{
	if (!m_tree || !m_model)
	{
		return;
	}
	const QSignalBlocker guard(m_model);
	for (int r = 0; r < m_model->rowCount(); ++r)
	{
		QStandardItem* root = m_model->item(r);
		if (!root)
		{
			continue;
		}
		const QString id = root->data(kRoleDocumentId).toString();
		const bool show = !documentId.isEmpty() && id == documentId;
		m_tree->setRowHidden(r, QModelIndex(), !show);
	}
	setActiveDocument(documentId);
}

bool BackendUnitsTreeBinder::hasDocument(const QString& documentId) const
{
	return !documentId.isEmpty() && m_documentRoots.contains(documentId);
}

void BackendUnitsTreeBinder::patchObjectVisible(const QString& documentId, const QString& backendId, bool visible)
{
	if (QStandardItem* item = findBackendItem(documentId, backendId))
	{
		const QSignalBlocker guard(m_model);
		item->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
	}
}

void BackendUnitsTreeBinder::addAnnotationItem(const QString& documentId, const QString& annotationId,
											   const QString& displayText, bool visible)
{
	QStandardItem* group = m_annotationGroups.value(documentId, nullptr);
	if (!group || annotationId.isEmpty())
	{
		return;
	}
	QStandardItem* item = makeLabeledItem(displayText, kItemTypeAnnotation, documentId, true);
	item->setData(annotationId, kRoleAnnotationId);
	item->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
	group->appendRow(item);
	expandItem(group);
}

void BackendUnitsTreeBinder::removeAnnotationItem(const QString& documentId, const QString& annotationId)
{
	QStandardItem* group = m_annotationGroups.value(documentId, nullptr);
	if (!group)
	{
		return;
	}
	for (int i = 0; i < group->rowCount(); ++i)
	{
		QStandardItem* child = group->child(i);
		if (child && child->data(kRoleAnnotationId).toString() == annotationId)
		{
			group->removeRow(i);
			return;
		}
	}
}

void BackendUnitsTreeBinder::setAnnotationItemVisible(const QString& documentId, const QString& annotationId,
													  bool visible)
{
	QStandardItem* group = m_annotationGroups.value(documentId, nullptr);
	if (!group)
	{
		return;
	}
	for (int i = 0; i < group->rowCount(); ++i)
	{
		QStandardItem* child = group->child(i);
		if (child && child->data(kRoleAnnotationId).toString() == annotationId)
		{
			const QSignalBlocker guard(m_model);
			child->setCheckState(visible ? Qt::Checked : Qt::Unchecked);
			return;
		}
	}
}

void BackendUnitsTreeBinder::setCurrentBackendItem(const QString& documentId, const QString& backendId, bool scroll)
{
	if (!m_tree)
	{
		return;
	}
	QStandardItem* item = findBackendItem(documentId, backendId);
	if (!item)
	{
		return;
	}
	const QModelIndex index = item->index();
	m_tree->setCurrentIndex(index);
	if (scroll)
	{
		m_tree->scrollTo(index);
	}
}
