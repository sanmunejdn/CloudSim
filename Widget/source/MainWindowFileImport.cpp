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

bool MainWindow::registerExistingBackendObject(std::shared_ptr<BackendDataBase> backendObject, const QString& sourcePath,
	const QString& typeName, const QString& persistedId, bool selectInTree, const QString& parentId)
{
	DocumentPage* doc = currentPage();
	if (!doc)
	{
		return false;
	}
	if (!backendObject)
	{
		return false;
	}
	if (!persistedId.isEmpty())
	{
		backendObject->setId(persistedId.toStdString());
	}
	if (!doc->backend().registerData(backendObject))
	{
		QMessageBox::warning(this, QStringLiteral("Backend Register"),
			QStringLiteral("Failed to register backend object."));
		return false;
	}
	const QString id = QString::fromStdString(backendObject->id());
	doc->backendSourcePath()[id] = sourcePath;
	doc->backendSourceType()[id] = typeName;
	if (!parentId.isEmpty())
	{
		if (!doc->backend().attachChild(parentId.toStdString(), id.toStdString()))
		{
			doc->backend().unregisterData(id.toStdString());
			QMessageBox::warning(this, QStringLiteral("Backend Relation"),
				QStringLiteral("Failed to link backend relation (cycle or invalid parent)."));
			return false;
		}
	}
	doc->backendParentId()[id] = parentId;
	if (OsgWidget* osg = doc->osgWidget())
	{
		osg->setBackendParent(id.toStdString(), parentId.toStdString());
	}
	applyHierarchyFollowBinding(doc, id.toStdString(), parentId.toStdString());
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(QStringLiteral("Backend object registered: %1").arg(QString::fromStdString(backendObject->name())));
	}
	refreshBackendTree();
	if (selectInTree)
	{
		focusBackendInTree(backendObject);
	}
	return true;
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

void MainWindow::onUrdfImportRequested(const QString& urdfPath)
{
	if (urdfPath.isEmpty())
	{
		return;
	}
	if (!currentOsgWidget())
	{
		QMessageBox::warning(this, QStringLiteral("URDF"), QStringLiteral("No active 3D view."));
		return;
	}
	MainWindowImportCaptureRenderController controller;
	(void)controller.registerUrdfRobot(*this, urdfPath, false);
}
