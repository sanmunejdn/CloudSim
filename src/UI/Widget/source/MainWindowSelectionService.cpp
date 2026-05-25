#include "MainWindowSelectionService.h"

#include <memory>
#include <vector>

#include <QList>
#include <QSignalBlocker>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "BackendSceneDocumentFacade.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendHierarchyModel.h"
#include "DocumentHostEvents.h"
#include "DocumentPage.h"
#include "MainWindow.h"
#include "MainWindowObjectRepository.h"
#include "MainWindow_p.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"
#include "RunInfoPage.h"
#include "SimulationCommandWidget.h"

using namespace mainwindow_detail;

void MainWindowSelectionService::clearSelection(MainWindow& mainWindow, bool clearTreeSelection)
{
	clearBackendObjectSelection(mainWindow, clearTreeSelection);
	mainWindow.m_activeInstructionForProperty.reset();
	if (mainWindow.simulationCommandPage())
	{
		mainWindow.simulationCommandPage()->clearInstructionSelection();
	}
	if (OsgWidget* osg = mainWindow.currentOsgWidget())
	{
		osg->clearInstructionPoseAxes();
	}
	mainWindow.updatePropertyPanel(nullptr);
}

void MainWindowSelectionService::clearBackendObjectSelection(MainWindow& mainWindow, bool clearTreeSelection)
{
	mainWindow.m_selectionState.clearBackendSelection();
	if (OsgWidget* osg = mainWindow.currentOsgWidget())
	{
		osg->setSelectionActive(false);
	}
	if (clearTreeSelection && mainWindow.m_backendTree)
	{
		mainWindow.m_backendTree->clearSelection();
	}
}

std::shared_ptr<BackendDataBase> MainWindowSelectionService::selectedBackendData(MainWindow& mainWindow)
{
	if (!mainWindow.m_selectionState.hasBackendSelection())
	{
		return nullptr;
	}
	const std::shared_ptr<BackendDataBase> data =
		MainWindowObjectRepository::findById(mainWindow, mainWindow.m_selectionState.selectedBackendId());
	if (data)
	{
		return data;
	}
	mainWindow.m_selectionState.clearBackendSelection();
	return nullptr;
}

MainWindowSelectionService::SelectionSnapshot MainWindowSelectionService::currentSelection(MainWindow& mainWindow)
{
	SelectionSnapshot snapshot;
	snapshot.backendId = mainWindow.m_selectionState.selectedBackendId();
	snapshot.data = selectedBackendData(mainWindow);
	if (!snapshot.valid())
	{
		snapshot.kind = SelectedBackendKind::None;
		snapshot.hasGeometry = false;
		return snapshot;
	}
	if (std::dynamic_pointer_cast<PointCloudBackendData>(snapshot.data))
	{
		snapshot.kind = SelectedBackendKind::PointCloud;
	}
	else if (std::dynamic_pointer_cast<MeshBackendData>(snapshot.data))
	{
		snapshot.kind = SelectedBackendKind::Mesh;
	}
	else
	{
		snapshot.kind = SelectedBackendKind::Other;
	}
	snapshot.hasGeometry = snapshot.data->hasGeometry();
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
	OsgWidget* osg = mainWindow.currentOsgWidget();
	if (!osg || !osg->hasImportedContent())
	{
		return;
	}
	osg->setSelectionActive(true);

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

	for (const std::shared_ptr<BackendDataBase>& data : MainWindowObjectRepository::listAll(mainWindow))
	{
		if (!data || !data->hasGeometry())
		{
			continue;
		}
		if (preferredKind == SelectedBackendKind::PointCloud
			&& !std::dynamic_pointer_cast<PointCloudBackendData>(data))
		{
			continue;
		}
		if (preferredKind == SelectedBackendKind::Mesh
			&& !std::dynamic_pointer_cast<MeshBackendData>(data))
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		mainWindow.m_selectionState.setSelectedBackendId(id);
		if (selectBackendById(mainWindow, id, false))
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
	if (OsgWidget* osg = mainWindow.currentOsgWidget())
	{
		osg->clearInstructionPoseAxes();
	}

	OsgWidget* osg = mainWindow.currentOsgWidget();

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
	mainWindow.m_selectionState.setSelectedBackendId(id);
	DocumentPage* doc = mainWindow.currentPage();
	const std::shared_ptr<BackendDataBase> data = selectedBackendData(mainWindow);
	const bool rowVisible = current->checkState(0) != Qt::Unchecked;
	const bool urdfLinkMesh = doc && doc->hasRobotSimulationContext() && doc->robotLinkBackendIds().contains(id);

	if (osg)
	{
		const std::string idStd = id.toStdString();
		if (doc)
		{
			doc->sceneFacade().entity(idStd).setVisible(rowVisible);
		}
		else
		{
			osg->setBackendObjectVisible(idStd, rowVisible);
		}
		osg->setSelectionActive(data != nullptr);
		if (data && doc)
		{
			doc->sceneFacade().ensureSelectionVisualForBackend(*data, urdfLinkMesh);
		}
	}
	if (doc)
	{
		cloudsim::host::publishSelectionChanged(*doc, id, cloudsim::core::SelectionSource::Tree);
	}
}

void MainWindowSelectionService::handleBackendTreeItemChanged(
	MainWindow& mainWindow,
	QTreeWidgetItem* item,
	int column)
{
	OsgWidget* const osg = mainWindow.currentOsgWidget();
	DocumentPage* const doc = mainWindow.currentPage();
	if (!item || column != 0 || !osg)
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
				osg->setBackendObjectVisible(id.toStdString(), visible);
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
			mainWindow.updatePropertyPanel(nullptr);
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
	osg->setAnnotationVisible(annotationId, visible);
}

void MainWindowSelectionService::handleOsgBackendObjectPicked(MainWindow& mainWindow, const QString& backendId)
{
	if (!mainWindow.m_backendTree || backendId.isEmpty())
	{
		return;
	}
	if (mainWindow.m_selectionState.selectedBackendId() == backendId)
	{
		return;
	}
	(void)selectBackendById(mainWindow, backendId, true);
}

