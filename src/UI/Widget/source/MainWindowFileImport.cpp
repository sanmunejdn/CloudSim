#include "MainWindow.h"

#include <memory>

#include <QFileDialog>
#include <QFileInfo>
#include <QMessageBox>
#include <QStringList>
#include <QTreeWidget>
#include <QTreeWidgetItem>

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentPage.h"
#include "MainWindow_p.h"
#include "MainWindowSelectionService.h"
#include "MeshBackendData.h"
#include "OsgWidget.h"
#include "PointCloudBackendData.h"
#include "RunInfoPage.h"
#include "MainWindowImportCaptureRenderController.h"

using namespace mainwindow_detail;

void MainWindow::focusBackendInTree(const std::shared_ptr<BackendDataBase>& backendObject)
{
	if (!backendObject || !m_backendTree || !m_backendRootItem)
	{
		return;
	}
	const QString idQs = QString::fromStdString(backendObject->id());
	(void)MainWindowSelectionService::selectBackendById(*this, idQs, true);
	updatePropertyPanel(backendObject);
}

bool MainWindow::registerBackendObject(const QString& filePath, const QString& typeName, bool isPointCloud, bool quietUi)
{
	MainWindowImportCaptureRenderController controller;
	return controller.registerBackendObject(*this, filePath, typeName, isPointCloud, quietUi);
}

void MainWindow::onOpenModel()
{
	const QString filter =
		QStringLiteral("Model Files (*.obj *.stl *.ply *.off *.dxf *.dae *.3ds *.fbx *.step *.stp *.igs *.iges);;All Files (*.*)");
	const QString filePath = QFileDialog::getOpenFileName(this, QStringLiteral("Open Model"), QString(), filter);
	if (filePath.isEmpty())
	{
		return;
	}

	if (!currentOsgWidget())
	{
		QMessageBox::warning(this, QStringLiteral("Import Model"), QStringLiteral("No active 3D view."));
		return;
	}

	registerBackendObject(filePath, QStringLiteral("Model"), false);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("Model opened: %1").arg(filePath));
	}
}

void MainWindow::onOpenPointCloud()
{
	const QString filter =
		QStringLiteral("Point Cloud Files (*.ply *.laz *.las *.xyz);;All Files (*.*)");
	const QString filePath = QFileDialog::getOpenFileName(this, QStringLiteral("Open Point Cloud"), QString(), filter);
	if (filePath.isEmpty())
	{
		return;
	}

	if (!currentOsgWidget())
	{
		QMessageBox::warning(this, QStringLiteral("Import Point Cloud"), QStringLiteral("No active 3D view."));
		return;
	}

	registerBackendObject(filePath, QStringLiteral("PointCloud"), true);
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("Point cloud opened: %1").arg(filePath));
	}
}
