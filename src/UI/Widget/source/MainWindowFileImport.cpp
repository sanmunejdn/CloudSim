#include "MainWindow.h"

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QMessageBox>
#include <QStringList>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

#include "DocumentPage.h"
#include "MainWindow_p.h"
#include "MainWindowSelectionService.h"
#include "RunInfoPage.h"
#include "WidgetRenderAccess.h"
#include "MainWindowImportCaptureRenderController.h"

using namespace mainwindow_detail;

namespace
{

bool isMeshQualityPromptExtension(const QString& extLower)
{
	return extLower == QLatin1String("obj") || extLower == QLatin1String("stl") || extLower == QLatin1String("ply")
		|| extLower == QLatin1String("off");
}

int promptMeshImportQuality(MainWindow& mw, const QString& filePath)
{
	const QFileInfo fileInfo(filePath);
	if (!isMeshQualityPromptExtension(fileInfo.suffix().toLower()))
	{
		return mw.meshImportQuality();
	}

	QDialog dialog(&mw);
	dialog.setWindowTitle(QStringLiteral("Mesh Import Quality"));
	auto* layout = new QVBoxLayout(&dialog);
	auto* form = new QFormLayout();
	auto* qualityCombo = new QComboBox(&dialog);
	qualityCombo->addItem(QStringLiteral("Coarse"), 0);
	qualityCombo->addItem(QStringLiteral("Medium"), 1);
	qualityCombo->addItem(QStringLiteral("Fine"), 2);
	const int current = mw.meshImportQuality();
	const int idx = qualityCombo->findData(current);
	qualityCombo->setCurrentIndex(idx >= 0 ? idx : 1);
	form->addRow(QStringLiteral("Triangle density:"), qualityCombo);
	layout->addLayout(form);
	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
	layout->addWidget(buttons);
	QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
	QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
	if (dialog.exec() != QDialog::Accepted)
	{
		return -1;
	}
	const int quality = qualityCombo->currentData().toInt();
	mw.setMeshImportQuality(quality);
	return quality;
}

} // namespace

void MainWindow::focusBackendInTreeLocal(const QString& backendId)
{
	if (backendId.isEmpty() || !m_backendTree || !m_backendRootItem)
	{
		return;
	}
	(void)MainWindowSelectionService::selectBackendById(*this, backendId, true);
	updatePropertyPanel(backendId);
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

	if (!renderWidgetFromPage(currentPage()))
	{
		QMessageBox::warning(this, QStringLiteral("Import Model"), QStringLiteral("No active 3D view."));
		return;
	}

	const int quality = promptMeshImportQuality(*this, filePath);
	if (quality < 0)
	{
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

	if (!renderWidgetFromPage(currentPage()))
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
