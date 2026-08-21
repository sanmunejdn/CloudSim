/// @file MainWindowSelectionService.cpp
/// @brief 选择与 gizmo 联动

#include "MainWindowSelectionService.h"

#include "BackendHierarchyModel.h"
#include "CoreEvents.h"
#include "CoreTypes.h"
#include "DocumentHostEvents.h"
#include "DocumentPage.h"
#include "IDataService.h"
#include "IRenderView.h"
#include "MainWindow.h"
#include "MainWindowObjectRepository.h"
#include "MainWindow_p.h"
#include "RunInfoPage.h"
#include "SelectionVisualService.h"
#include "SimulationCommandWidget.h"
#include "WidgetRenderAccess.h"

#include <QList>
#include <QItemSelectionModel>
#include <QModelIndex>
#include <QSignalBlocker>
#include <QStandardItem>
#include <QStandardItemModel>
#include <QTreeView>
#include <memory>
#include <vector>

using namespace mainwindow_detail;

namespace
{
cloudsim::core::IRenderView* activeRenderView(MainWindow& mainWindow)
{
	return renderViewFromPage(mainWindow.currentPage());
}

DocumentPage* pageFromTreeItem(MainWindow& mainWindow, const QStandardItem* item)
{
	if (!item)
	{
		return nullptr;
	}
	const QString docId = item->data(kRoleDocumentId).toString();
	if (!docId.isEmpty())
	{
		return mainWindow.pageByDocumentId(docId);
	}
	return mainWindow.currentPage();
}
} // namespace

QString MainWindowSelectionService::selectionRootBackendId(MainWindow& mainWindow, const QString& backendId)
{
	DocumentPage* doc = mainWindow.currentPage();
	if (!doc)
	{
		return backendId;
	}
	return doc->selectionRootBackendId(backendId);
}

QString MainWindowSelectionService::resolveObjectSelectionBackendId(MainWindow& mainWindow, const QString& backendId,
																	const cloudsim::core::SelectionSource source)
{
	DocumentPage* doc = mainWindow.currentPage();
	if (!doc)
	{
		return backendId;
	}
	return doc->resolveObjectSelectionBackendId(backendId, source, mainWindow.m_selectionState.selectedBackendId());
}

void MainWindowSelectionService::applyBackendSelection(MainWindow& mainWindow, const QString& backendId,
													   const cloudsim::core::SelectionSource source,
													   const bool rowVisible)
{
	if (backendId.isEmpty())
	{
		return;
	}
	(void)rowVisible; // 选中不改写可见性；可见性仅由树勾选/Data 驱动
	const QString effectiveId = resolveObjectSelectionBackendId(mainWindow, backendId, source);
	mainWindow.m_selectionState.setSelectedBackendId(effectiveId);
	DocumentPage* doc = mainWindow.currentPage();
	cloudsim::core::IRenderView* rv = activeRenderView(mainWindow);
	const QString gizmoId = doc ? doc->robotGizmoAnchorBackendId(effectiveId) : effectiveId;
	const bool urdfLinkMesh = doc && doc->hasRobotSimulationContext() && doc->robotLinkBackendIds().contains(gizmoId);
	const bool hasSelection = doc && doc->data().isValid(effectiveId);

	if (rv)
	{
		rv->setSelectionActive(hasSelection);
		if (source == cloudsim::core::SelectionSource::OsgPick)
		{
			rv->setObjectSelectionMode(true);
		}
		if (hasSelection && doc)
		{
			// 3D 拾取命中常是连杆；gizmo 必须挂整机锚点（与树选一致），勿停在命中 id
			const QString visualId = (gizmoId != effectiveId && doc->data().isValid(gizmoId)) ? gizmoId : effectiveId;
			cloudsim::host::SelectionVisualService::ensureSelectionVisual(*doc, visualId, urdfLinkMesh);
		}
	}
	if (doc)
	{
		cloudsim::host::publishSelectionChanged(*doc, effectiveId, source);
	}
}

void MainWindowSelectionService::clearSelection(MainWindow& mainWindow, bool clearTreeSelection)
{
	clearBackendObjectSelection(mainWindow, clearTreeSelection);
	mainWindow.m_activeInstructionForProperty.reset();
	if (mainWindow.simulationCommandPage())
	{
		mainWindow.simulationCommandPage()->clearInstructionSelection();
	}
	if (cloudsim::core::IRenderView* rv = activeRenderView(mainWindow))
	{
		rv->clearInstructionPoseAxes();
	}
	mainWindow.updatePropertyPanel(QString());
}

void MainWindowSelectionService::clearBackendObjectSelection(MainWindow& mainWindow, bool clearTreeSelection)
{
	mainWindow.m_selectionState.clearBackendSelection();
	if (cloudsim::core::IRenderView* rv = activeRenderView(mainWindow))
	{
		rv->setSelectionActive(false);
	}
	if (clearTreeSelection && mainWindow.m_backendTree)
	{
		mainWindow.m_backendTree->clearSelection();
	}
}

MainWindowSelectionService::SelectionSnapshot MainWindowSelectionService::currentSelection(MainWindow& mainWindow)
{
	SelectionSnapshot snapshot;
	snapshot.backendId = mainWindow.m_selectionState.selectedBackendId();
	if (snapshot.backendId.isEmpty())
	{
		snapshot.kind = SelectedBackendKind::None;
		snapshot.hasGeometry = false;
		return snapshot;
	}
	DocumentPage* page = mainWindow.currentPage();
	if (!page || !page->data().isValid(snapshot.backendId))
	{
		snapshot.backendId.clear();
		snapshot.kind = SelectedBackendKind::None;
		snapshot.hasGeometry = false;
		return snapshot;
	}
	const cloudsim::core::GeometryKind gk = page->data().geometryKind(snapshot.backendId);
	if (gk == cloudsim::core::GeometryKind::Points)
	{
		snapshot.kind = SelectedBackendKind::PointCloud;
	}
	else if (gk == cloudsim::core::GeometryKind::Mesh)
	{
		snapshot.kind = SelectedBackendKind::Mesh;
	}
	else
	{
		snapshot.kind = SelectedBackendKind::Other;
	}
	if (const auto dto = MainWindowObjectRepository::findSnapshot(mainWindow, snapshot.backendId))
	{
		snapshot.hasGeometry = dto->hasGeometry;
	}
	return snapshot;
}

bool MainWindowSelectionService::selectBackendById(MainWindow& mainWindow, const QString& backendId, bool scrollToItem)
{
	if (!mainWindow.m_backendTree || backendId.isEmpty() || !mainWindow.unitsTreeBinder())
	{
		return false;
	}
	BackendUnitsTreeBinder* binder = mainWindow.unitsTreeBinder();
	QStandardItem* item = nullptr;
	QString docId;
	if (DocumentPage* cur = mainWindow.currentPage())
	{
		docId = cur->documentId();
		item = binder->findBackendItem(docId, backendId);
	}
	if (!item)
	{
		item = binder->findBackendItemAnyDocument(backendId);
		if (item)
		{
			docId = item->data(kRoleDocumentId).toString();
		}
	}
	if (!item)
	{
		return false;
	}
	if (!docId.isEmpty() && mainWindow.currentPage() && mainWindow.currentPage()->documentId() != docId)
	{
		mainWindow.activateDocumentById(docId);
	}
	binder->setCurrentBackendItem(docId, backendId, scrollToItem);
	return true;
}

void MainWindowSelectionService::ensureBackendForPickMode(MainWindow& mainWindow,
														  const SelectedBackendKind preferredKind)
{
	cloudsim::core::IRenderView* rv = activeRenderView(mainWindow);
	if (!rv || !rv->hasImportedContent())
	{
		return;
	}
	rv->setSelectionActive(true);

	const SelectionSnapshot current = currentSelection(mainWindow);
	if (current.valid() && current.hasGeometry)
	{
		if (preferredKind == SelectedBackendKind::PointCloud && current.kind == SelectedBackendKind::PointCloud)
		{
			return;
		}
		if (preferredKind == SelectedBackendKind::Mesh && current.kind == SelectedBackendKind::Mesh)
		{
			return;
		}
	}

	DocumentPage* page = mainWindow.currentPage();
	if (!page)
	{
		return;
	}

	for (const cloudsim::core::BackendObjectDto& dto : MainWindowObjectRepository::listSnapshots(mainWindow))
	{
		if (!dto.hasGeometry)
		{
			continue;
		}
		const cloudsim::core::GeometryKind gk = page->data().geometryKind(dto.id);
		if (preferredKind == SelectedBackendKind::PointCloud && gk != cloudsim::core::GeometryKind::Points)
		{
			continue;
		}
		if (preferredKind == SelectedBackendKind::Mesh && gk != cloudsim::core::GeometryKind::Mesh)
		{
			continue;
		}
		mainWindow.m_selectionState.setSelectedBackendId(dto.id);
		if (selectBackendById(mainWindow, dto.id, false))
		{
			handleBackendTreeSelectionChanged(mainWindow);
		}
		return;
	}
}

void MainWindowSelectionService::handleBackendTreeSelectionChanged(MainWindow& mainWindow)
{
	if (!mainWindow.m_backendTree || mainWindow.isUnitsTreeStructureMuted())
	{
		return;
	}
	mainWindow.m_activeInstructionForProperty.reset();
	if (mainWindow.simulationCommandPage())
	{
		mainWindow.simulationCommandPage()->clearInstructionSelection();
	}

	const QModelIndexList selected = mainWindow.m_backendTree->selectionModel()
										 ? mainWindow.m_backendTree->selectionModel()->selectedRows()
										 : QModelIndexList();
	if (selected.isEmpty())
	{
		clearSelection(mainWindow, false);
		return;
	}

	BackendUnitsTreeBinder* binder = mainWindow.unitsTreeBinder();
	QStandardItem* current = binder ? binder->itemFromIndex(selected.first()) : nullptr;
	if (!current)
	{
		clearSelection(mainWindow, false);
		return;
	}
	const int itemType = current->data(kRoleItemType).toInt();
	if (itemType != kItemTypeBackend)
	{
		clearSelection(mainWindow, false);
		return;
	}

	// activate/rebuild 会销毁树节点，必须先拷贝
	const QString docId = current->data(kRoleDocumentId).toString();
	const QString id = current->data(kRoleBackendId).toString();
	const bool rowVisible = current->checkState() != Qt::Unchecked;

	if (!docId.isEmpty() &&
		(!mainWindow.currentPage() || mainWindow.currentPage()->documentId() != docId))
	{
		mainWindow.activateDocumentById(docId);
	}

	cloudsim::core::IRenderView* rv = activeRenderView(mainWindow);
	if (rv)
	{
		rv->clearInstructionPoseAxes();
	}

	applyBackendSelection(mainWindow, id, cloudsim::core::SelectionSource::Tree, rowVisible);
	const QString effectiveId = resolveObjectSelectionBackendId(mainWindow, id, cloudsim::core::SelectionSource::Tree);
	if (effectiveId != id)
	{
		const QSignalBlocker guard(mainWindow.m_backendTree);
		(void)selectBackendById(mainWindow, effectiveId, false);
	}
}

void MainWindowSelectionService::handleBackendTreeItemChanged(MainWindow& mainWindow, QStandardItem* item)
{
	if (!item || mainWindow.isUnitsTreeStructureMuted())
	{
		return;
	}

	const int itemType = item->data(kRoleItemType).toInt();
	// 文档根/分组仅样式变更也会走 itemChanged，切勿因此 activate
	if (itemType != kItemTypeBackend && itemType != kItemTypeAnnotation)
	{
		return;
	}

	// activate/rebuild 会销毁节点，先拷贝
	const QString documentId = item->data(kRoleDocumentId).toString();
	const QString backendId = item->data(kRoleBackendId).toString();
	const QString annotationId = item->data(kRoleAnnotationId).toString();
	const Qt::CheckState checkState = item->checkState();

	DocumentPage* doc = nullptr;
	if (!documentId.isEmpty())
	{
		doc = mainWindow.pageByDocumentId(documentId);
	}
	if (!doc)
	{
		doc = pageFromTreeItem(mainWindow, item);
	}
	if (!doc)
	{
		return;
	}
	if (doc != mainWindow.currentPage())
	{
		mainWindow.activateDocumentById(doc->documentId());
		doc = mainWindow.pageByDocumentId(documentId);
		if (!doc)
		{
			return;
		}
	}
	cloudsim::core::IRenderView* rv = renderViewFromPage(doc);
	if (!rv)
	{
		return;
	}

	if (itemType == kItemTypeBackend)
	{
		QStringList idsToUpdate;
		const std::vector<std::string> subtree = doc->hierarchyModel().subtreeIds(backendId.toStdString());
		if (subtree.empty())
		{
			idsToUpdate.append(backendId);
		}
		else
		{
			for (const std::string& id : subtree)
			{
				idsToUpdate.append(QString::fromStdString(id));
			}
		}
		const bool visible = checkState != Qt::Unchecked;
		const QSignalBlocker guard(mainWindow.m_backendTree);
		doc->setBackendsVisible(idsToUpdate, visible);
		BackendUnitsTreeBinder* binder = mainWindow.unitsTreeBinder();
		for (const QString& id : idsToUpdate)
		{
			if (binder)
			{
				binder->patchObjectVisible(documentId, id, visible);
			}
		}
		if (!visible && mainWindow.m_selectionState.hasBackendSelection() &&
			idsToUpdate.contains(mainWindow.m_selectionState.selectedBackendId()))
		{
			clearBackendObjectSelection(mainWindow, false);
			mainWindow.updatePropertyPanel(QString());
		}
		if (mainWindow.m_runInfoPage)
		{
			mainWindow.m_runInfoPage->appendInfo(
				QStringLiteral("Scene content %1.").arg(visible ? QStringLiteral("shown") : QStringLiteral("hidden")));
		}
		return;
	}

	if (itemType != kItemTypeAnnotation)
	{
		return;
	}
	const bool visible = checkState == Qt::Checked;
	rv->setAnnotationVisible(annotationId, visible);
}

void MainWindowSelectionService::handleOsgBackendObjectPicked(MainWindow& mainWindow, const QString& backendId)
{
	if (backendId.isEmpty())
	{
		return;
	}
	const QString effectiveId =
		resolveObjectSelectionBackendId(mainWindow, backendId, cloudsim::core::SelectionSource::OsgPick);
	const bool sameSelection = mainWindow.m_selectionState.selectedBackendId() == effectiveId;

	if (!sameSelection)
	{
		mainWindow.m_activeInstructionForProperty.reset();
		if (mainWindow.simulationCommandPage())
		{
			mainWindow.simulationCommandPage()->clearInstructionSelection();
		}
		if (cloudsim::core::IRenderView* rv = activeRenderView(mainWindow))
		{
			rv->clearInstructionPoseAxes();
		}
	}

	bool rowVisible = true;
	if (mainWindow.unitsTreeBinder())
	{
		QStandardItem* item = nullptr;
		if (DocumentPage* cur = mainWindow.currentPage())
		{
			item = mainWindow.unitsTreeBinder()->findBackendItem(cur->documentId(), effectiveId);
		}
		if (!item)
		{
			item = mainWindow.unitsTreeBinder()->findBackendItemAnyDocument(effectiveId);
		}
		if (!item)
		{
			(void)selectBackendById(mainWindow, effectiveId, false);
			if (DocumentPage* cur = mainWindow.currentPage())
			{
				item = mainWindow.unitsTreeBinder()->findBackendItem(cur->documentId(), effectiveId);
			}
		}
		if (item)
		{
			rowVisible = item->checkState() != Qt::Unchecked;
		}
	}

	applyBackendSelection(mainWindow, backendId, cloudsim::core::SelectionSource::OsgPick, rowVisible);

	if (!sameSelection && mainWindow.m_backendTree)
	{
		const QSignalBlocker guard(mainWindow.m_backendTree);
		(void)selectBackendById(mainWindow, effectiveId, true);
	}
}
