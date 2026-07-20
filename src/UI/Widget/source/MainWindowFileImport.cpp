/// @file MainWindowFileImport.cpp
/// @brief MainWindowFileImport 实现

#include "DocumentPage.h"
#include "MainWindow.h"
#include "MainWindowImportCaptureRenderController.h"
#include "MainWindowSelectionService.h"
#include "MainWindow_p.h"
#include "RunInfoPage.h"
#include "WidgetRenderAccess.h"

#include "BackendFileImport.h"
#include "FrameBackendData.h"

#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QLineEdit>
#include <QMessageBox>
#include <QStringList>
#include <QTreeWidget>
#include <QTreeWidgetItem>
#include <QVBoxLayout>

using namespace mainwindow_detail;

namespace
{
bool isMeshQualityPromptExtension(const QString& extLower)
{
	return extLower == QLatin1String("obj") || extLower == QLatin1String("stl") || extLower == QLatin1String("ply") ||
		   extLower == QLatin1String("off");
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

bool MainWindow::registerBackendObject(const QString& filePath, const QString& typeName, bool isPointCloud,
									   bool quietUi)
{
	MainWindowImportCaptureRenderController controller;
	return controller.registerBackendObject(*this, filePath, typeName, isPointCloud, quietUi);
}

void MainWindow::onOpenModel()
{
	const QString filter = QStringLiteral(
		"Model Files (*.obj *.stl *.ply *.off *.dxf *.dae *.3ds *.fbx *.step *.stp *.igs *.iges);;All Files (*.*)");
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
	const QString filter = QStringLiteral("Point Cloud Files (*.ply *.laz *.las *.xyz);;All Files (*.*)");
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

void MainWindow::onCreateCoordinateFrame()
{
	cloudsim::host::DocumentHost* host = currentDocumentHost();
	if (!host || !renderWidgetFromPage(currentPage()))
	{
		QMessageBox::warning(
			this,
			i18n(QStringLiteral("Coordinate Frame"), QStringLiteral("坐标系")),
			i18n(QStringLiteral("No active document / 3D view."), QStringLiteral("没有活动文档或三维视图。")));
		return;
	}

	QDialog dialog(this);
	dialog.setWindowTitle(i18n(QStringLiteral("Create Coordinate Frame"), QStringLiteral("新建坐标系")));
	auto* layout = new QVBoxLayout(&dialog);
	auto* form = new QFormLayout();
	auto* nameEdit = new QLineEdit(&dialog);
	nameEdit->setText(i18n(QStringLiteral("CoordinateFrame"), QStringLiteral("坐标系")));
	auto* xSpin = new QDoubleSpinBox(&dialog);
	auto* ySpin = new QDoubleSpinBox(&dialog);
	auto* zSpin = new QDoubleSpinBox(&dialog);
	auto* rxSpin = new QDoubleSpinBox(&dialog);
	auto* rySpin = new QDoubleSpinBox(&dialog);
	auto* rzSpin = new QDoubleSpinBox(&dialog);
	for (QDoubleSpinBox* s : {xSpin, ySpin, zSpin})
	{
		s->setRange(-1.0e6, 1.0e6);
		s->setDecimals(3);
		s->setSuffix(QStringLiteral(" mm"));
		s->setValue(0.0);
	}
	for (QDoubleSpinBox* s : {rxSpin, rySpin, rzSpin})
	{
		s->setRange(-360.0, 360.0);
		s->setDecimals(3);
		s->setSuffix(QStringLiteral(" deg"));
		s->setValue(0.0);
	}
	form->addRow(i18n(QStringLiteral("Name"), QStringLiteral("名称")), nameEdit);
	form->addRow(QStringLiteral("X"), xSpin);
	form->addRow(QStringLiteral("Y"), ySpin);
	form->addRow(QStringLiteral("Z"), zSpin);
	form->addRow(QStringLiteral("Rx"), rxSpin);
	form->addRow(QStringLiteral("Ry"), rySpin);
	form->addRow(QStringLiteral("Rz"), rzSpin);
	layout->addLayout(form);
	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
	layout->addWidget(buttons);
	QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
	QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
	if (dialog.exec() != QDialog::Accepted)
	{
		return;
	}

	QString name = nameEdit->text().trimmed();
	if (name.isEmpty())
	{
		name = i18n(QStringLiteral("CoordinateFrame"), QStringLiteral("坐标系"));
	}

	auto frame = std::make_shared<FrameBackendData>();
	frame->setName(name.toStdString());
	frame->setPose(BackendVec3{xSpin->value(), ySpin->value(), zSpin->value()});
	frame->setRotation(BackendVec3{rxSpin->value(), rySpin->value(), rzSpin->value()});

	QString err;
	if (!cloudsim::host::registerAdoptedFrameAndLoadScene(*host, frame, QStringLiteral("CoordinateFrame"), QString(),
														  false, &err))
	{
		QMessageBox::warning(
			this,
			i18n(QStringLiteral("Coordinate Frame"), QStringLiteral("坐标系")),
			err.isEmpty() ? i18n(QStringLiteral("Failed to create frame."), QStringLiteral("创建坐标系失败。")) : err);
		return;
	}

	focusBackendInTreeAfterImport(QString::fromStdString(frame->id()));
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("Coordinate frame created: %1").arg(name),
				 QStringLiteral("已创建坐标系：%1").arg(name)));
	}
}

