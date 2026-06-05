#include "MainWindowSelectionService.h"

#include <memory>
#include <vector>

#include <QList>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "BackendSceneDocumentFacade.h"
#include "CoreEvents.h"
#include "CoreTypes.h"
#include "DocumentHostEvents.h"
#include "DocumentPage.h"
#include "IDataService.h"
#include "IRenderView.h"
#include "BackendHierarchyModel.h"
#include "MainWindow.h"
#include "MainWindowObjectRepository.h"
#include "MainWindow_p.h"
#include "SelectionVisualService.h"
#include "WidgetRenderAccess.h"
#include "RunInfoPage.h"
#include "SimulationCommandWidget.h"

using namespace mainwindow_detail;

namespace
{
cloudsim::core::IRenderView* activeRenderView(MainWindow& mainWindow)
{
	return renderViewFromPage(mainWindow.currentPage());
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

void MainWindowSelectionService::applyBackendSelection(
	MainWindow& mainWindow,
	const QString& backendId,
	const cloudsim::core::SelectionSource source,
	const bool rowVisible)
{
	if (backendId.isEmpty())
	{
		return;
	}
	const QString effectiveId = selectionRootBackendId(mainWindow, backendId);
	mainWindow.m_selectionState.setSelectedBackendId(effectiveId);
	DocumentPage* doc = mainWindow.currentPage();
	cloudsim::core::IRenderView* rv = activeRenderView(mainWindow);
	const QString gizmoId = doc ? doc->robotGizmoAnchorBackendId(effectiveId) : effectiveId;
	const bool urdfLinkMesh =
		doc && doc->hasRobotSimulationContext() && doc->robotLinkBackendIds().contains(gizmoId);
	const bool hasSelection = doc && doc->data().isValid(effectiveId);

	if (rv)
	{
		if (doc)
		{
			doc->sceneFacade().entity(effectiveId.toStdString()).setVisible(rowVisible);
		}
		else
		{
			rv->setVisible(effectiveId, rowVisible);
		}
		rv->setSelectionActive(hasSelection);
		if (source == cloudsim::core::SelectionSource::OsgPick)
		{
			rv->setObjectSelectionMode(true);
		}
		if (hasSelection && doc)
		{
			if (gizmoId != effectiveId && doc->data().isValid(gizmoId))
			{
				cloudsim::host::SelectionVisualService::ensureSelectionVisual(*doc, gizmoId, urdfLinkMesh);
			}
			else
			{
				cloudsim::host::SelectionVisualService::ensureSelectionVisual(*doc, effectiveId, urdfLinkMesh);
			}
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
	if (!mainWindow.m_backendTree || backendId.isEmpty())
	{
		return false;
	}
	QTreeWidgetItem* item = mainWindow.m_backendTreeItemsById.value(backendId, nullptr);
	if (!item)
	{
		const QList<QTreeWidgetItem*> allItems =
			mainWindow.m_backendTree->findItems(QStringLiteral("*"), Qt::MatchWildcard | Qt::MatchRecursive, 0);
		for (QTreeWidgetItem* candidate : allItems)
		{
			if (!candidate || candidate->data(0, kRoleItemType).toInt() != kItemTypeBackend)
			{
				continue;
			}
			if (candidate->data(0, kRoleBackendId).toString() != backendId)
			{
				continue;
			}
			item = candidate;
			mainWindow.m_backendTreeItemsById.insert(backendId, candidate);
			break;
		}
	}
	if (!item)
	{
		return false;
	}
	mainWindow.m_backendTree->setCurrentItem(item);
	if (scrollToItem)
	{
		mainWindow.m_backendTree->scrollToItem(item);
	}
	return true;
}

void MainWindowSelectionService::ensureBackendForPickMode(
	MainWindow& mainWindow,
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
	if (!mainWindow.m_backendTree)
	{
		return;
	}
	mainWindow.m_activeInstructionForProperty.reset();
	if (mainWindow.simulationCommandPage())
	{
		mainWindow.simulationCommandPage()->clearInstructionSelection();
	}
	cloudsim::core::IRenderView* rv = activeRenderView(mainWindow);
	if (rv)
	{
		rv->clearInstructionPoseAxes();
	}

	const QList<QTreeWidgetItem*> selected = mainWindow.m_backendTree->selectedItems();
	if (selected.isEmpty() || selected.first() == mainWindow.m_backendRootItem)
	{
		clearSelection(mainWindow, false);
		return;
	}

	const QTreeWidgetItem* current = selected.first();
	if (current->data(0, kRoleItemType).toInt() != kItemTypeBackend)
	{
		clearSelection(mainWindow, false);
		return;
	}
	const QString id = current->data(0, kRoleBackendId).toString();
	const bool rowVisible = current->checkState(0) != Qt::Unchecked;
	applyBackendSelection(mainWindow, id, cloudsim::core::SelectionSource::Tree, rowVisible);
	const QString effectiveId = selectionRootBackendId(mainWindow, id);
	if (effectiveId != id)
	{
		const QSignalBlocker guard(mainWindow.m_backendTree);
		(void)selectBackendById(mainWindow, effectiveId, false);
	}
}

void MainWindowSelectionService::handleBackendTreeItemChanged(
	MainWindow& mainWindow,
	QTreeWidgetItem* item,
	int column)
{
	cloudsim::core::IRenderView* rv = activeRenderView(mainWindow);
	DocumentPage* const doc = mainWindow.currentPage();
	if (!item || column != 0 || !rv)
	{
		return;
	}

	const int itemType = item->data(0, kRoleItemType).toInt();
	if (itemType == kItemTypeBackend)
	{
		const QString backendId = item->data(0, kRoleBackendId).toString();
		QVector<QString> idsToUpdate;
		if (doc)
		{
			const std::vector<std::string>& sub = doc->hierarchyModel().subtreeIds(backendId.toStdString());
			if (sub.empty())
			{
				idsToUpdate.append(backendId);
			}
			else
			{
				idsToUpdate.reserve(static_cast<int>(sub.size()));
				for (const std::string& id : sub)
				{
					idsToUpdate.append(QString::fromStdString(id));
				}
			}
		}
		else
		{
			idsToUpdate.append(backendId);
		}
		if (idsToUpdate.isEmpty())
		{
			return;
		}
		const bool visible = item->checkState(0) != Qt::Unchecked;
		const QSignalBlocker guard(mainWindow.m_backendTree);
		if (doc)
		{
			std::vector<std::string> idsStd;
			idsStd.reserve(static_cast<std::size_t>(idsToUpdate.size()));
			for (const QString& id : idsToUpdate)
			{
				idsStd.push_back(id.toStdString());
			}
			doc->sceneFacade().setBackendsVisible(idsStd, visible);
		}
		else
		{
			for (const QString& id : idsToUpdate)
			{
				rv->setVisible(id, visible);
			}
		}
		for (const QString& id : idsToUpdate)
		{
			if (id == backendId)
			{
				continue;
			}
			QTreeWidgetItem* const childItem = mainWindow.m_backendTreeItemsById.value(id, nullptr);
			if (!childItem)
			{
				continue;
			}
			childItem->setCheckState(0, visible ? Qt::Checked : Qt::Unchecked);
		}
		if (!visible && mainWindow.m_selectionState.hasBackendSelection()
			&& idsToUpdate.contains(mainWindow.m_selectionState.selectedBackendId()))
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
	const QString annotationId = item->data(0, kRoleAnnotationId).toString();
	const bool visible = item->checkState(0) == Qt::Checked;
	rv->setAnnotationVisible(annotationId, visible);
}

void MainWindowSelectionService::handleOsgBackendObjectPicked(MainWindow& mainWindow, const QString& backendId)
{
	if (backendId.isEmpty())
	{
		return;
	}
	const QString effectiveId = selectionRootBackendId(mainWindow, backendId);
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
	if (mainWindow.m_backendTree)
	{
		QTreeWidgetItem* item = mainWindow.m_backendTreeItemsById.value(effectiveId, nullptr);
		if (!item)
		{
			(void)selectBackendById(mainWindow, effectiveId, false);
			item = mainWindow.m_backendTreeItemsById.value(effectiveId, nullptr);
		}
		if (item)
		{
			rowVisible = item->checkState(0) != Qt::Unchecked;
		}
	}

	// 同一机器人再次点连杆时 pickAndActivate 会临时激活该连杆；须始终把 gizmo 归到锚点
	applyBackendSelection(mainWindow, backendId, cloudsim::core::SelectionSource::OsgPick, rowVisible);

	if (!sameSelection && mainWindow.m_backendTree)
	{
		const QSignalBlocker guard(mainWindow.m_backendTree);
		(void)selectBackendById(mainWindow, effectiveId, true);
	}
}
