/// @file BackendUnitsTreeBinder.cpp
/// @brief Units 树文档作用域绑定

#include "BackendUnitsTreeBinder.h"

#include "MainWindow_p.h"

#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItem>

using namespace mainwindow_detail;

BackendUnitsTreeBinder::BackendUnitsTreeBinder(QTreeWidget* tree) : m_tree(tree) {}

void BackendUnitsTreeBinder::setAnnotationGroupLabel(const QString& label)
{
	m_annotationGroupLabel = label;
	for (auto it = m_annotationGroups.begin(); it != m_annotationGroups.end(); ++it)
	{
		if (it.value())
		{
			it.value()->setText(0, m_annotationGroupLabel);
		}
	}
}

QTreeWidgetItem* BackendUnitsTreeBinder::documentRoot(const QString& documentId) const
{
	return m_documentRoots.value(documentId, nullptr);
}

QTreeWidgetItem* BackendUnitsTreeBinder::annotationGroup(const QString& documentId) const
{
	return m_annotationGroups.value(documentId, nullptr);
}

QTreeWidgetItem* BackendUnitsTreeBinder::findBackendItem(const QString& documentId, const QString& backendId) const
{
	return m_backendItems.value(DocObjKey(documentId, backendId), nullptr);
}

QTreeWidgetItem* BackendUnitsTreeBinder::findBackendItemAnyDocument(const QString& backendId) const
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

void BackendUnitsTreeBinder::applyActiveStyle(QTreeWidgetItem* docRoot, bool isActive) const
{
	if (!docRoot || !m_tree)
	{
		return;
	}
	QFont font = docRoot->font(0);
	const bool wantBold = isActive;
	if (font.bold() == wantBold)
	{
		return;
	}
	font.setBold(wantBold);
	// setFont 会触发 itemChanged，须阻断以免与 activateDocument 重入
	const QSignalBlocker guard(m_tree);
	docRoot->setFont(0, font);
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

void BackendUnitsTreeBinder::clearDocumentChildrenKeepRoot(QTreeWidgetItem* docRoot, const QString& documentId)
{
	if (!docRoot)
	{
		return;
	}
	forgetDocumentIndexes(documentId);
	const QList<QTreeWidgetItem*> kids = docRoot->takeChildren();
	qDeleteAll(kids);
}

QTreeWidgetItem* BackendUnitsTreeBinder::ensureDocumentRoot(const QString& documentId, const QString& title,
															bool isActive)
{
	if (!m_tree || documentId.isEmpty())
	{
		return nullptr;
	}
	QTreeWidgetItem* root = m_documentRoots.value(documentId, nullptr);
	if (!root)
	{
		root = new QTreeWidgetItem(QStringList() << title);
		root->setData(0, kRoleItemType, kItemTypeDocument);
		root->setData(0, kRoleDocumentId, documentId);
		m_tree->addTopLevelItem(root);
		m_documentRoots.insert(documentId, root);
	}
	else
	{
		root->setText(0, title);
	}
	applyActiveStyle(root, isActive);
	root->setExpanded(true);
	return root;
}

void BackendUnitsTreeBinder::syncDocument(const BackendUnitsDisplayDocument& doc)
{
	if (!m_tree || doc.documentId.isEmpty())
	{
		return;
	}
	const QSignalBlocker guard(m_tree);
	QTreeWidgetItem* docRoot = ensureDocumentRoot(doc.documentId, doc.title, doc.isActive);
	if (!docRoot)
	{
		return;
	}
	clearDocumentChildrenKeepRoot(docRoot, doc.documentId);

	auto* annGroup = new QTreeWidgetItem(QStringList() << m_annotationGroupLabel);
	annGroup->setData(0, kRoleItemType, kItemTypeAnnotationGroup);
	annGroup->setData(0, kRoleDocumentId, doc.documentId);
	docRoot->addChild(annGroup);
	annGroup->setExpanded(true);
	m_annotationGroups.insert(doc.documentId, annGroup);

	for (const BackendUnitsDisplayAnnotation& a : doc.annotations)
	{
		if (a.id.isEmpty())
		{
			continue;
		}
		auto* item = new QTreeWidgetItem(QStringList() << a.displayText);
		item->setData(0, kRoleItemType, kItemTypeAnnotation);
		item->setData(0, kRoleDocumentId, doc.documentId);
		item->setData(0, kRoleAnnotationId, a.id);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setCheckState(0, a.visible ? Qt::Checked : Qt::Unchecked);
		annGroup->addChild(item);
	}

	QHash<QString, QTreeWidgetItem*> idToItem;
	idToItem.reserve(doc.objectOrder.size());
	for (const QString& id : doc.objectOrder)
	{
		const BackendUnitsDisplayObject obj = doc.objects.value(id);
		if (obj.id.isEmpty())
		{
			continue;
		}
		const QString nodeText = QStringLiteral("%1 [%2]").arg(obj.name).arg(obj.id);
		auto* item = new QTreeWidgetItem(QStringList() << nodeText);
		item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
		item->setData(0, kRoleItemType, kItemTypeBackend);
		item->setData(0, kRoleDocumentId, doc.documentId);
		item->setData(0, kRoleBackendId, obj.id);
		item->setCheckState(0, obj.visible ? Qt::Checked : Qt::Unchecked);
		idToItem.insert(obj.id, item);
		m_backendItems.insert(DocObjKey(doc.documentId, obj.id), item);
	}

	for (const QString& id : doc.objectOrder)
	{
		const BackendUnitsDisplayObject obj = doc.objects.value(id);
		QTreeWidgetItem* item = idToItem.value(id, nullptr);
		if (!item)
		{
			continue;
		}
		QTreeWidgetItem* parentItem = docRoot;
		if (!obj.primaryParentId.isEmpty())
		{
			if (QTreeWidgetItem* p = idToItem.value(obj.primaryParentId, nullptr))
			{
				parentItem = p;
			}
		}
		parentItem->addChild(item);
	}
}

void BackendUnitsTreeBinder::removeDocument(const QString& documentId)
{
	if (!m_tree || documentId.isEmpty())
	{
		return;
	}
	QTreeWidgetItem* root = m_documentRoots.take(documentId);
	forgetDocumentIndexes(documentId);
	if (!root)
	{
		return;
	}
	const int idx = m_tree->indexOfTopLevelItem(root);
	if (idx >= 0)
	{
		delete m_tree->takeTopLevelItem(idx);
	}
	else
	{
		delete root;
	}
}

void BackendUnitsTreeBinder::retainOnlyDocument(const QString& documentId)
{
	if (!m_tree)
	{
		return;
	}
	const QSignalBlocker guard(m_tree);
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
	if (!m_tree)
	{
		return;
	}
	const QSignalBlocker guard(m_tree);
	for (auto it = m_documentRoots.begin(); it != m_documentRoots.end(); ++it)
	{
		if (!it.value())
		{
			continue;
		}
		const bool show = !documentId.isEmpty() && it.key() == documentId;
		it.value()->setHidden(!show);
	}
	setActiveDocument(documentId);
}

bool BackendUnitsTreeBinder::hasDocument(const QString& documentId) const
{
	return !documentId.isEmpty() && m_documentRoots.contains(documentId);
}

void BackendUnitsTreeBinder::patchObjectVisible(const QString& documentId, const QString& backendId, bool visible)
{
	if (QTreeWidgetItem* item = findBackendItem(documentId, backendId))
	{
		const QSignalBlocker guard(m_tree);
		item->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
	}
}

void BackendUnitsTreeBinder::addAnnotationItem(const QString& documentId, const QString& annotationId,
											   const QString& displayText, bool visible)
{
	QTreeWidgetItem* group = m_annotationGroups.value(documentId, nullptr);
	if (!group || annotationId.isEmpty())
	{
		return;
	}
	auto* item = new QTreeWidgetItem(QStringList() << displayText);
	item->setData(0, kRoleItemType, kItemTypeAnnotation);
	item->setData(0, kRoleDocumentId, documentId);
	item->setData(0, kRoleAnnotationId, annotationId);
	item->setFlags(item->flags() | Qt::ItemIsUserCheckable);
	item->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
	group->addChild(item);
	group->setExpanded(true);
}

void BackendUnitsTreeBinder::removeAnnotationItem(const QString& documentId, const QString& annotationId)
{
	QTreeWidgetItem* group = m_annotationGroups.value(documentId, nullptr);
	if (!group)
	{
		return;
	}
	for (int i = 0; i < group->childCount(); ++i)
	{
		QTreeWidgetItem* child = group->child(i);
		if (child && child->data(0, kRoleAnnotationId).toString() == annotationId)
		{
			delete group->takeChild(i);
			return;
		}
	}
}

void BackendUnitsTreeBinder::setAnnotationItemVisible(const QString& documentId, const QString& annotationId,
													  bool visible)
{
	QTreeWidgetItem* group = m_annotationGroups.value(documentId, nullptr);
	if (!group)
	{
		return;
	}
	for (int i = 0; i < group->childCount(); ++i)
	{
		QTreeWidgetItem* child = group->child(i);
		if (child && child->data(0, kRoleAnnotationId).toString() == annotationId)
		{
			const QSignalBlocker guard(m_tree);
			child->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
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
	QTreeWidgetItem* item = findBackendItem(documentId, backendId);
	if (!item)
	{
		return;
	}
	m_tree->setCurrentItem(item);
	if (scroll)
	{
		m_tree->scrollToItem(item);
	}
}
