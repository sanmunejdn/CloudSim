#pragma once

#include "widget_global.h"

#include <memory>
#include <QString>

class QString;
class MainWindow;
class BackendDataBase;
class QTreeWidgetItem;

/// Centralized selection orchestration for MainWindow tree/OSG pick flows.
class WIDGET_EXPORT MainWindowSelectionService
{
public:
	enum class SelectedBackendKind
	{
		None,
		PointCloud,
		Mesh,
		Other
	};

	struct SelectionSnapshot
	{
		QString backendId;
		std::shared_ptr<BackendDataBase> data;
		SelectedBackendKind kind = SelectedBackendKind::None;
		bool hasGeometry = false;
		bool valid() const { return !backendId.isEmpty() && static_cast<bool>(data); }
	};

	static void clearSelection(MainWindow& mainWindow, bool clearTreeSelection);
	static void clearBackendObjectSelection(MainWindow& mainWindow, bool clearTreeSelection);
	static void handleBackendTreeSelectionChanged(MainWindow& mainWindow);
	static void handleBackendTreeItemChanged(MainWindow& mainWindow, QTreeWidgetItem* item, int column);
	static void handleOsgBackendObjectPicked(MainWindow& mainWindow, const QString& backendId);
	static std::shared_ptr<BackendDataBase> selectedBackendData(MainWindow& mainWindow);
	static SelectionSnapshot currentSelection(MainWindow& mainWindow);
	static bool selectBackendById(MainWindow& mainWindow, const QString& backendId, bool scrollToItem = true);
};

