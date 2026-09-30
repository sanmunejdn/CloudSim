/// @file MainWindowBackendTree.cpp
/// @brief 后端对象树

#include "../RobotWidget/inc/RobotSimulationController.h"
#include "BackendHierarchyModel.h"
#include "BackendProjectObjectIo.h"
#include "BackendTypeIds.h"
#include "BackendUnitsDisplayForest.h"
#include "CoreTypes.h"
#include "DocumentPage.h"
#include "DocumentPointCloudOps.h"
#include "IDataService.h"
#include "IRenderView.h"
#include "MainWindow.h"
#include "MainWindowSelectionService.h"
#include "MainWindow_p.h"
#include "RunInfoPage.h"
#include "SceneHierarchyCanvasWidget.h"
#include "WidgetRenderAccess.h"

#include <QAction>
#include <QFileDialog>
#include <QHash>
#include <QList>
#include <QMenu>
#include <QMessageBox>
#include <QPoint>
#include <QSet>
#include <QStandardItem>
#include <QStringList>
#include <QTreeView>

using namespace mainwindow_detail;

namespace
{
QStringList collectSubtreeBackendIds(const DocumentPage& doc, const QString& rootBackendId)
{
	QStringList ids;
	if (rootBackendId.isEmpty())
	{
		return ids;
	}
	const std::string rootStd = rootBackendId.toStdString();
	const std::vector<std::string> subtree = doc.hierarchyModel().subtreeIds(rootStd);
	if (subtree.empty() && const_cast<DocumentPage&>(doc).data().isValid(rootBackendId))
	{
		ids.append(rootBackendId);
	}
	else
	{
		for (const std::string& id : subtree)
		{
			ids.append(QString::fromStdString(id));
		}
	}
	return ids;
}

} // namespace

void MainWindow::beginBackendTreeEventRefreshSuppress()
{
	++m_backendTreeEventRefreshSuppress;
}

void MainWindow::endBackendTreeEventRefreshSuppress()
{
	if (m_backendTreeEventRefreshSuppress > 0)
	{
		--m_backendTreeEventRefreshSuppress;
	}
}

MainWindow::ScopedBackendTreeRefreshSuppress::ScopedBackendTreeRefreshSuppress(MainWindow& mw) : m_mw(mw)
{
	m_mw.beginBackendTreeEventRefreshSuppress();
}

MainWindow::ScopedBackendTreeRefreshSuppress::~ScopedBackendTreeRefreshSuppress()
{
	m_mw.endBackendTreeEventRefreshSuppress();
}

void MainWindow::focusBackendInTreeAfterImport(const QString& backendId)
{
	focusBackendInTreeLocal(backendId);
}

void MainWindow::markUnitsDocumentDirty(const QString& documentId)
{
	if (!documentId.isEmpty())
	{
		m_unitsTreeDirtyDocumentIds.insert(documentId);
	}
}

void MainWindow::rebuildUnitsDocument(const QString& documentId)
{
	if (!m_unitsTreeBinder || documentId.isEmpty() || !m_documentTabs)
	{
		return;
	}
	DocumentPage* page = pageByDocumentId(documentId);
	if (!page)
	{
		m_unitsTreeBinder->removeDocument(documentId);
		m_unitsTreeDirtyDocumentIds.remove(documentId);
		return;
	}
	// 非当前文档：保留已构建子树，仅置脏，切回再 sync
	if (page != currentPage())
	{
		markUnitsDocumentDirty(documentId);
		return;
	}
	const int tabIndex = m_documentTabs->indexOf(page);
	const QString title =
		tabIndex >= 0 ? m_documentTabs->tabText(tabIndex) : i18n(QStringLiteral("Untitled"), QStringLiteral("未命名"));
	const bool cacheHit =
		m_unitsTreeBinder->hasDocument(documentId) && !m_unitsTreeDirtyDocumentIds.contains(documentId);
	++m_unitsTreeStructureMute;
	m_unitsTreeBinder->showOnlyDocument(documentId);
	if (cacheHit)
	{
		if (QStandardItem* root = m_unitsTreeBinder->documentRoot(documentId))
		{
			root->setText(title);
		}
		--m_unitsTreeStructureMute;
		return;
	}
	QVector<cloudsim::core::AnnotationSnapshotDto> anns;
	if (cloudsim::core::IRenderView* rv = renderViewFromPage(page))
	{
		anns = rv->annotationSnapshots();
	}
	const BackendUnitsDisplayDocument displayDoc =
		BackendUnitsDisplayForest::buildDocument(documentId, title, true, page->data().listObjectSnapshots(), anns);
	const QString selectedBackendId = m_selectionState.selectedBackendId();
	m_unitsTreeBinder->syncDocument(displayDoc);
	if (!selectedBackendId.isEmpty())
	{
		m_unitsTreeBinder->setCurrentBackendItem(documentId, selectedBackendId, false);
	}
	--m_unitsTreeStructureMute;
	m_unitsTreeDirtyDocumentIds.remove(documentId);
	// 树节点勾选已由 snapshot 写入；全量 visibility walk 留给显式 patch / 用户勾选
	cloudsim::host::syncOsgBackendParentsFromBackend(*page);
}

void MainWindow::applyCurrentDocumentVisibilityToScene()
{
	DocumentPage* page = currentPage();
	if (!page)
	{
		return;
	}
	cloudsim::core::IRenderView& rv = page->render();
	for (const cloudsim::core::BackendObjectDto& dto : page->data().listObjectSnapshots())
	{
		if (dto.id.isEmpty())
		{
			continue;
		}
		rv.setVisible(dto.id, dto.visible);
	}
	rv.requestRedraw();
}

void MainWindow::refreshBackendTree()
{
	if (!m_backendTree || !m_unitsTreeBinder || !m_documentTabs)
	{
		return;
	}

	m_unitsTreeBinder->setAnnotationGroupLabel(i18n(QStringLiteral("Annotations"), QStringLiteral("注释")));

	if (DocumentPage* cur = currentPage())
	{
		markUnitsDocumentDirty(cur->documentId());
		rebuildUnitsDocument(cur->documentId());
	}
	else
	{
		m_unitsTreeBinder->showOnlyDocument(QString());
	}
	refreshOsgSceneTree();
}

void MainWindow::refreshOsgSceneTree()
{
	if (!m_osgSceneTree)
	{
		return;
	}
	// Scene 调试页未显示时跳过整图快照，切 Tab 常见卡点
	if (m_unitDockTabs && m_unitDockTabs->currentWidget() != m_osgSceneTree)
	{
		return;
	}
	DocumentPage* page = currentPage();
	if (!page)
	{
		m_osgSceneTree->setEmptyHint(i18n(QStringLiteral("No scene"), QStringLiteral("无场景")));
		return;
	}
	const auto snapshot = page->render().sceneGraphSnapshot(256);
	if (snapshot.className.isEmpty())
	{
		m_osgSceneTree->setEmptyHint(i18n(QStringLiteral("No scene"), QStringLiteral("无场景")));
		return;
	}
	m_osgSceneTree->setSceneGraph(snapshot);
}

void MainWindow::onBackendTreeSelectionChanged()
{
	MainWindowSelectionService::handleBackendTreeSelectionChanged(*this);
}

void MainWindow::onOsgBackendObjectPicked(const QString& backendId)
{
	MainWindowSelectionService::handleOsgBackendObjectPicked(*this, backendId);
}

void MainWindow::onAnnotationCreated(const QString& annotationId, const QString& displayText)
{
	DocumentPage* doc = nullptr;
	if (m_documentTabs)
	{
		for (int i = 0; i < m_documentTabs->count(); ++i)
		{
			auto* p = qobject_cast<DocumentPage*>(m_documentTabs->widget(i));
			if (p && renderWidgetFromPage(p) == sender())
			{
				doc = p;
				break;
			}
		}
	}
	if (!doc || !m_unitsTreeBinder)
	{
		return;
	}
	m_unitsTreeBinder->addAnnotationItem(doc->documentId(), annotationId, displayText, true);
	if (doc == currentPage())
	{
		refreshOsgSceneTree();
	}
}

void MainWindow::onAnnotationRemoved(const QString& annotationId)
{
	DocumentPage* doc = nullptr;
	if (m_documentTabs)
	{
		for (int i = 0; i < m_documentTabs->count(); ++i)
		{
			auto* p = qobject_cast<DocumentPage*>(m_documentTabs->widget(i));
			if (p && renderWidgetFromPage(p) == sender())
			{
				doc = p;
				break;
			}
		}
	}
	if (!doc || !m_unitsTreeBinder)
	{
		return;
	}
	m_unitsTreeBinder->removeAnnotationItem(doc->documentId(), annotationId);
	if (doc == currentPage())
	{
		refreshOsgSceneTree();
	}
}

void MainWindow::onAnnotationVisibilityChanged(const QString& annotationId, bool visible)
{
	DocumentPage* doc = nullptr;
	if (m_documentTabs)
	{
		for (int i = 0; i < m_documentTabs->count(); ++i)
		{
			auto* p = qobject_cast<DocumentPage*>(m_documentTabs->widget(i));
			if (p && renderWidgetFromPage(p) == sender())
			{
				doc = p;
				break;
			}
		}
	}
	if (!doc || !m_unitsTreeBinder)
	{
		return;
	}
	m_unitsTreeBinder->setAnnotationItemVisible(doc->documentId(), annotationId, visible);
}

void MainWindow::onBackendTreeContextMenu(const QPoint& pos)
{
	if (!m_backendTree || !m_unitsTreeBinder)
	{
		return;
	}
	QStandardItem* item = m_unitsTreeBinder->itemFromIndex(m_backendTree->indexAt(pos));
	if (!item)
	{
		return;
	}

	const QString documentId = item->data(kRoleDocumentId).toString();
	DocumentPage* doc = pageByDocumentId(documentId);
	if (!doc)
	{
		doc = currentPage();
	}
	if (doc && doc != currentPage())
	{
		activateDocumentById(doc->documentId());
	}
	cloudsim::core::IRenderView* rv = renderViewFromPage(doc);
	if (!rv)
	{
		return;
	}

	const int itemType = item->data(kRoleItemType).toInt();
	if (itemType == kItemTypeAnnotationGroup)
	{
		QMenu menu(this);
		QAction* clearAll =
			menu.addAction(i18n(QStringLiteral("Clear All Annotations"), QStringLiteral("清空全部注释")));
		QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
		if (action == clearAll)
		{
			rv->clearAllAnnotations();
			markUnitsDocumentDirty(doc->documentId());
			rebuildUnitsDocument(doc->documentId());
			refreshOsgSceneTree();
			if (m_runInfoPage)
			{
				m_runInfoPage->appendInfo(QStringLiteral("All annotations cleared."));
			}
		}
		return;
	}

	if (itemType == kItemTypeBackend)
	{
		const bool visible = item->checkState() == Qt::Checked;
		const QString backendId = item->data(kRoleBackendId).toString();
		QMenu menu(this);
		QAction* toggle = menu.addAction(visible ? i18n(QStringLiteral("Hide Object"), QStringLiteral("隐藏对象"))
												 : i18n(QStringLiteral("Show Object"), QStringLiteral("显示对象")));
		QAction* focusView = menu.addAction(i18n(QStringLiteral("Focus View"), QStringLiteral("聚焦显示")));
		QAction* exportObj = nullptr;
		if (doc && doc->data().isValid(backendId))
		{
			const cloudsim::core::BackendObjectDto dto = doc->data().objectSnapshot(backendId);
			if (dto.hasGeometry)
			{
				if (backend_type::isPointCloudClassName(dto.className.toStdString()))
				{
					exportObj =
						menu.addAction(i18n(QStringLiteral("Export Point Cloud…"), QStringLiteral("导出点云…")));
				}
				else if (backend_type::isBrepWorkpieceClassName(dto.className.toStdString()))
				{
					exportObj = menu.addAction(i18n(QStringLiteral("Export STEP…"), QStringLiteral("导出 STEP…")));
				}
			}
		}
		QAction* deleteObj = menu.addAction(i18n(QStringLiteral("Delete"), QStringLiteral("删除")));
		QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
		if (action == toggle)
		{
			item->setCheckState(visible ? Qt::Unchecked : Qt::Checked);
		}
		else if (action == focusView)
		{
			rv->focusCameraOnBackend(backendId);
		}
		else if (action == exportObj)
		{
			exportBackendObjectFromTree(backendId);
		}
		else if (action == deleteObj)
		{
			const QMessageBox::StandardButton r = QMessageBox::question(
				this, i18n(QStringLiteral("Delete object"), QStringLiteral("删除对象")),
				i18n(QStringLiteral("Delete this object and all child parts? This cannot be undone."),
					 QStringLiteral("删除该对象及其子部件？此操作无法撤销。")),
				QMessageBox::Yes | QMessageBox::No, QMessageBox::No);
			if (r == QMessageBox::Yes)
			{
				removeBackendObjectFromDocument(backendId);
			}
		}
		return;
	}

	if (itemType != kItemTypeAnnotation)
	{
		return;
	}
	const QString annotationId = item->data(kRoleAnnotationId).toString();
	const bool visible = item->checkState() == Qt::Checked;

	QMenu menu(this);
	QAction* toggle = menu.addAction(visible ? i18n(QStringLiteral("Hide Annotation"), QStringLiteral("隐藏注释"))
											 : i18n(QStringLiteral("Show Annotation"), QStringLiteral("显示注释")));
	QAction* remove = menu.addAction(i18n(QStringLiteral("Delete Annotation"), QStringLiteral("删除注释")));
	QAction* action = menu.exec(m_backendTree->viewport()->mapToGlobal(pos));
	if (!action)
	{
		return;
	}
	if (action == toggle)
	{
		rv->setAnnotationVisible(annotationId, !visible);
	}
	else if (action == remove)
	{
		rv->removeAnnotation(annotationId);
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
	const QStringList removed = collectSubtreeBackendIds(*doc, backendId);
	if (removed.isEmpty())
	{
		return;
	}
	QString unregErr;
	if (!doc->data().unregisterSubtree(backendId, &unregErr))
	{
		if (m_runInfoPage && !unregErr.isEmpty())
		{
			m_runInfoPage->appendWarning(unregErr);
		}
		return;
	}
	for (const QString& rid : removed)
	{
		doc->clearRobotSimulationIfContains(rid);
	}
	if (m_robotSimulation && m_robotSimulation->programExecutor().isRunning() && !doc->hasRobotSimulationContext())
	{
		stopRobotSimulation();
	}
	MainWindowSelectionService::clearSelection(*this, true);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("Removed backend object(s): %1").arg(removed.join(QStringLiteral(", "))),
				 QStringLiteral("已删除后端对象: %1").arg(removed.join(QStringLiteral(", ")))));
	}
	refreshSimulationJointListFromCurrentDoc();
}

void MainWindow::exportBackendObjectFromTree(const QString& backendId)
{
	DocumentPage* doc = currentPage();
	if (!doc || backendId.isEmpty() || !doc->data().isValid(backendId))
	{
		return;
	}
	const cloudsim::core::BackendObjectDto dto = doc->data().objectSnapshot(backendId);
	if (!dto.hasGeometry)
	{
		return;
	}
	const bool isPointCloud = backend_type::isPointCloudClassName(dto.className.toStdString());
	const bool isBrep = backend_type::isBrepWorkpieceClassName(dto.className.toStdString());
	if (!isPointCloud && !isBrep)
	{
		return;
	}

	const QString baseName = dto.name.isEmpty() ? backendId : dto.name;
	QString savePath;
	if (isPointCloud)
	{
		savePath = QFileDialog::getSaveFileName(
			this, i18n(QStringLiteral("Export Point Cloud"), QStringLiteral("导出点云")),
			baseName + QStringLiteral(".ply"), QStringLiteral("PLY Files (*.ply);;All Files (*.*)"));
	}
	else
	{
		savePath = QFileDialog::getSaveFileName(this, i18n(QStringLiteral("Export STEP"), QStringLiteral("导出 STEP")),
												baseName + QStringLiteral(".step"),
												QStringLiteral("STEP Files (*.step *.stp);;All Files (*.*)"));
	}
	if (savePath.isEmpty())
	{
		return;
	}

	const std::string idUtf8 = backendId.toUtf8().constData();
	const std::string pathUtf8 = savePath.toUtf8().constData();
	std::string err;
	const bool ok = isPointCloud ? document_point_cloud_ops::exportPointCloudToPly(doc, idUtf8, pathUtf8, &err)
								 : document_point_cloud_ops::exportBrepToStep(doc, idUtf8, pathUtf8, &err);
	if (!m_runInfoPage)
	{
		return;
	}
	if (ok)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("Exported to %1").arg(savePath), QStringLiteral("已导出到 %1").arg(savePath)));
	}
	else
	{
		const QString errQs = err.empty() ? QStringLiteral("unknown error") : QString::fromStdString(err);
		m_runInfoPage->appendWarning(
			i18n(QStringLiteral("Export failed: %1").arg(errQs), QStringLiteral("导出失败: %1").arg(errQs)));
	}
}
