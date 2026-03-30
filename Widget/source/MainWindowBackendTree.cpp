#include "MainWindow.h"

#include <QAction>
#include <QList>
#include <QMenu>
#include <QMessageBox>
#include <QPoint>
#include <QHash>
#include <QStringList>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentPage.h"
#include "MainWindow_p.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"
#include "RunInfoPage.h"

#include <osg/Vec3f>

using namespace mainwindow_detail;

void MainWindow::refreshBackendTree()
{
	if (!m_backendTree || !m_backendRootItem)
	{
		return;
	}

	QString selectedBackendId;
	const QList<QTreeWidgetItem*> prevSel = m_backendTree->selectedItems();
	if (!prevSel.isEmpty())
	{
		QTreeWidgetItem* cur = prevSel.first();
		if (cur && cur->data(0, kRoleItemType).toInt() == kItemTypeBackend)
		{
			selectedBackendId = cur->data(0, kRoleBackendId).toString();
		}
	}

	m_backendRootItem->takeChildren();
	m_annotationRootItem = new QTreeWidgetItem(QStringList()
		<< i18n(QStringLiteral("Annotations"), QStringLiteral("\u6CE8\u91CA")));
	m_backendRootItem->addChild(m_annotationRootItem);
	m_annotationRootItem->setExpanded(true);
	const auto dataList = activeBackend().listData();
	QHash<QString, QTreeWidgetItem*> idToItem;
	QHash<QString, QString> idToParent;
	DocumentPage* page = currentPage();
	for (const auto& data : dataList)
	{
		if (!data)
		{
			continue;
		}

		const QString nodeText = QStringLiteral("%1 [%2]")
			.arg(QString::fromStdString(data->name()))
			.arg(QString::fromStdString(data->id()));
		auto* child = new QTreeWidgetItem(QStringList() << nodeText);
		child->setFlags(child->flags() | Qt::ItemIsUserCheckable);
		child->setData(0, kRoleItemType, kItemTypeBackend);
		child->setData(0, kRoleBackendId, QString::fromStdString(data->id()));
		child->setCheckState(0, Qt::Checked);
		const QString id = QString::fromStdString(data->id());
		idToItem.insert(id, child);
		idToParent.insert(id, page ? page->backendParentId().value(id) : QString());
	}
	for (auto it = idToItem.begin(); it != idToItem.end(); ++it)
	{
		const QString id = it.key();
		QTreeWidgetItem* item = it.value();
		const QString parentId = idToParent.value(id);
		QTreeWidgetItem* parentItem = idToItem.value(parentId, m_backendRootItem);
		if (!parentItem)
		{
			parentItem = m_backendRootItem;
		}
		parentItem->addChild(item);
	}

	if (OsgWidget* osg = currentOsgWidget())
	{
		const QList<OsgWidget::AnnotationSnapshot> snaps = osg->annotationSnapshots();
		for (const OsgWidget::AnnotationSnapshot& s : snaps)
		{
			const QString label = s.displayText.isEmpty() ? s.id : s.displayText;
			auto* item = new QTreeWidgetItem(QStringList() << label);
			item->setData(0, kRoleItemType, kItemTypeAnnotation);
			item->setData(0, kRoleAnnotationId, s.id);
			item->setCheckState(0, s.visible ? Qt::Checked : Qt::Unchecked);
			m_annotationRootItem->addChild(item);
		}
		if (!snaps.isEmpty())
		{
			m_annotationRootItem->setExpanded(true);
		}
	}

	m_backendRootItem->setExpanded(true);

	if (!selectedBackendId.isEmpty())
	{
		for (int i = 0; i < m_backendRootItem->childCount(); ++i)
		{
			QTreeWidgetItem* c = m_backendRootItem->child(i);
			if (!c || c == m_annotationRootItem)
			{
				continue;
			}
			if (c->data(0, kRoleItemType).toInt() == kItemTypeBackend
				&& c->data(0, kRoleBackendId).toString() == selectedBackendId)
			{
				m_backendTree->setCurrentItem(c);
				break;
			}
		}
	}
}

void MainWindow::onBackendTreeSelectionChanged()
{
	if (!m_backendTree)
	{
		return;
	}

	OsgWidget* osg = currentOsgWidget();

	const QList<QTreeWidgetItem*> selected = m_backendTree->selectedItems();
	if (selected.isEmpty() || selected.first() == m_backendRootItem)
	{
		if (osg)
		{
			osg->setSelectionActive(false);
		}
		updatePropertyPanel(nullptr);
		return;
	}

	const QTreeWidgetItem* current = selected.first();
	if (current->data(0, kRoleItemType).toInt() != kItemTypeBackend)
	{
		if (osg)
		{
			osg->setSelectionActive(false);
		}
		updatePropertyPanel(nullptr);
		return;
	}
	const QString id = current->data(0, kRoleBackendId).toString();
	const auto data = activeBackend().getData(id.toStdString());
	const bool rowVisible = current->checkState(0) != Qt::Unchecked;

	if (osg)
	{
		const std::string idStd = id.toStdString();
		osg->setBackendObjectVisible(idStd, rowVisible);
		osg->setSelectionActive(data != nullptr);
		if (data)
		{
			auto pointCloud = std::dynamic_pointer_cast<PointCloudBackendData>(data);
			if (pointCloud && !pointCloud->pointPositionsXyz().empty())
			{
				// Full load clears point annotations; only reload when the branch is missing.
				if (osg->hasBackendObjectBranch(idStd))
				{
					osg->syncSelectionFromBackend(*pointCloud);
				}
				else
				{
					QString geomErr;
					osg->loadPointCloudFromBackendData(*pointCloud, &geomErr, false);
				}
			}
			else if (auto mesh = std::dynamic_pointer_cast<MeshBackendData>(data))
			{
				if (!mesh->triangleSoup().empty())
				{
					if (osg->hasBackendObjectBranch(idStd))
					{
						osg->syncSelectionFromBackend(*mesh);
					}
					else
					{
						QString geomErr;
						osg->loadMeshFromBackendData(*mesh, &geomErr, false);
					}
				}
				else
				{
					// Parent/group node without direct geometry: keep this backend as active
					// and allow picking across all visible children.
					osg->syncSelectionForBackendId(idStd);
				}
			}
			else
			{
				// Generic backend with no direct render branch (e.g. assembly parent).
				osg->syncSelectionForBackendId(idStd);
			}
		}
	}
	updatePropertyPanel(data);
}

void MainWindow::onAnnotationCreated(const QString& annotationId, const QString& displayText)
{
	if (sender() != currentOsgWidget() || !m_annotationRootItem)
	{
		return;
	}
	auto* item = new QTreeWidgetItem(QStringList() << displayText);
	item->setData(0, kRoleItemType, kItemTypeAnnotation);
	item->setData(0, kRoleAnnotationId, annotationId);
	item->setCheckState(0, Qt::Checked);
	m_annotationRootItem->addChild(item);
	m_annotationRootItem->setExpanded(true);
}

void MainWindow::onAnnotationRemoved(const QString& annotationId)
{
	if (sender() != currentOsgWidget() || !m_annotationRootItem)
	{
		return;
	}
	for (int i = 0; i < m_annotationRootItem->childCount(); ++i)
	{
		QTreeWidgetItem* child = m_annotationRootItem->child(i);
		if (child && child->data(0, kRoleAnnotationId).toString() == annotationId)
		{
			delete m_annotationRootItem->takeChild(i);
			return;
		}
	}
}

void MainWindow::onAnnotationVisibilityChanged(const QString& annotationId, bool visible)
{
	if (sender() != currentOsgWidget() || !m_annotationRootItem)
	{
		return;
	}
	for (int i = 0; i < m_annotationRootItem->childCount(); ++i)
	{
		QTreeWidgetItem* child = m_annotationRootItem->child(i);
		if (child && child->data(0, kRoleAnnotationId).toString() == annotationId)
		{
			child->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
			return;
		}
	}
}

void MainWindow::onBackendTreeContextMenu(const QPoint& pos)
{
	OsgWidget* osg = currentOsgWidget();
	if (!m_backendTree || !osg)
	{
		return;
	}
	QTreeWidgetItem* item = m_backendTree->itemAt(pos);
	if (!item)
	{
		return;
	}

	if (item == m_annotationRootItem)
	{
		QMenu menu(this);
		QAction* clearAll = menu.addAction(i18n(QStringLiteral("Clear All Annotations"), QStringLiteral("\u6E05\u7A7A\u5168\u90E8\u6CE8\u91CA")));
		QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
		if (action == clearAll)
		{
			osg->clearAllAnnotations();
			if (m_runInfoPage)
			{
				m_runInfoPage->appendInfo(QStringLiteral("All annotations cleared."));
			}
		}
		return;
	}

	if (item->data(0, kRoleItemType).toInt() != kItemTypeAnnotation)
	{
		if (item->data(0, kRoleItemType).toInt() == kItemTypeBackend)
		{
			const bool visible = item->checkState(0) == Qt::Checked;
			QMenu menu(this);
			QAction* toggle = menu.addAction(visible
				? i18n(QStringLiteral("Hide Object"), QStringLiteral("\u9690\u85CF\u5BF9\u8C61"))
				: i18n(QStringLiteral("Show Object"), QStringLiteral("\u663E\u793A\u5BF9\u8C61")));
			QAction* focusView = menu.addAction(i18n(QStringLiteral("Focus View"), QStringLiteral("\u805A\u7126\u663E\u793A")));
			QAction* deleteObj = menu.addAction(i18n(QStringLiteral("Delete"), QStringLiteral("\u5220\u9664")));
			QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
			if (action == toggle)
			{
				item->setCheckState(0, visible ? Qt::Unchecked : Qt::Checked);
			}
			else if (action == focusView)
			{
				const QString id = item->data(0, kRoleBackendId).toString();
				osg->focusCameraOnBackend(id.toStdString());
			}
			else if (action == deleteObj)
			{
				const QString id = item->data(0, kRoleBackendId).toString();
				const QMessageBox::StandardButton r = QMessageBox::question(
					this,
					i18n(QStringLiteral("Delete object"), QStringLiteral("\u5220\u9664\u5BF9\u8C61")),
					i18n(QStringLiteral("Delete this object and all child parts? This cannot be undone."),
						QStringLiteral(
							"\u5220\u9664\u8BE5\u5BF9\u8C61\u53CA\u5176\u5B50\u90E8\u4EF6\uFF1F\u6B64\u64CD\u4F5C\u65E0\u6CD5\u64A4\u9500\u3002")),
					QMessageBox::Yes | QMessageBox::No,
					QMessageBox::No);
				if (r == QMessageBox::Yes)
				{
					removeBackendObjectFromDocument(id);
				}
			}
		}
		return;
	}
	const QString annotationId = item->data(0, kRoleAnnotationId).toString();
	const bool visible = item->checkState(0) == Qt::Checked;

	QMenu menu(this);
	QAction* toggle = menu.addAction(visible
		? i18n(QStringLiteral("Hide Annotation"), QStringLiteral("\u9690\u85CF\u6CE8\u91CA"))
		: i18n(QStringLiteral("Show Annotation"), QStringLiteral("\u663E\u793A\u6CE8\u91CA")));
	QAction* remove = menu.addAction(i18n(QStringLiteral("Delete Annotation"), QStringLiteral("\u5220\u9664\u6CE8\u91CA")));
	QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
	if (!action)
	{
		return;
	}
	if (action == toggle)
	{
		osg->setAnnotationVisible(annotationId, !visible);
	}
	else if (action == remove)
	{
		osg->removeAnnotation(annotationId);
		if (m_runInfoPage)
		{
			m_runInfoPage->appendInfo(QStringLiteral("Annotation removed: %1").arg(annotationId));
		}
	}
}

void MainWindow::removeBackendObjectFromDocument(const QString& backendId)
{
	DocumentPage* doc = currentPage();
	if (!doc || backendId.isEmpty())
	{
		return;
	}
	const QStringList removed = doc->removeBackendSubtree(backendId);
	if (removed.isEmpty())
	{
		return;
	}
	for (const QString& rid : removed)
	{
		doc->clearRobotSimulationIfContains(rid);
	}
	if (m_robotInstructionPlayback.isRunning() && !doc->hasRobotSimulationContext())
	{
		stopRobotSimulation();
	}
	if (OsgWidget* osg = doc->osgWidget())
	{
		for (const QString& id : removed)
		{
			osg->removeBackendObjectVisual(id.toStdString());
		}
	}
	updatePropertyPanel(nullptr);
	refreshBackendTree();
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("Removed backend object(s): %1").arg(removed.join(QStringLiteral(", "))),
				QStringLiteral("\u5DF2\u5220\u9664\u540E\u7AEF\u5BF9\u8C61: %1").arg(removed.join(QStringLiteral(", ")))));
	}
	refreshSimulationJointListFromCurrentDoc();
}
