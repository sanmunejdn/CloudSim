#ifndef WIDGET_MAINWINDOWSELECTIONSERVICE_H
#define WIDGET_MAINWINDOWSELECTIONSERVICE_H

/// @file MainWindowSelectionService.h
/// @brief MainWindow 树/OSG 拾取的选择编排

#include "widget_global.h"

#include <QString>

class QString;
class MainWindow;
class QTreeWidgetItem;

namespace cloudsim::core
{
enum class SelectionSource;
}

/// MainWindow 树/OSG 拾取的选择编排
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
		SelectedBackendKind kind = SelectedBackendKind::None;
		bool hasGeometry = false;
		bool valid() const { return !backendId.isEmpty(); }
	};

	static void clearSelection(MainWindow& mainWindow, bool clearTreeSelection);
	static void clearBackendObjectSelection(MainWindow& mainWindow, bool clearTreeSelection);
	static void handleBackendTreeSelectionChanged(MainWindow& mainWindow);
	static void handleBackendTreeItemChanged(MainWindow& mainWindow, QTreeWidgetItem* item, int column);
	static void handleOsgBackendObjectPicked(MainWindow& mainWindow, const QString& backendId);
	static SelectionSnapshot currentSelection(MainWindow& mainWindow);
	static bool selectBackendById(MainWindow& mainWindow, const QString& backendId, bool scrollToItem = true);
	/// 进入点/线/面拾取前：确保有带几何的匹配 backend 被选中
	static void ensureBackendForPickMode(MainWindow& mainWindow, SelectedBackendKind preferredKind);

private:
	static QString selectionRootBackendId(MainWindow& mainWindow, const QString& backendId);
	static void applyBackendSelection(MainWindow& mainWindow, const QString& backendId,
									  cloudsim::core::SelectionSource source, bool rowVisible);
};

#endif // WIDGET_MAINWINDOWSELECTIONSERVICE_H
