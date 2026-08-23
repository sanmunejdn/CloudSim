/// @file MainWindowFileImport.cpp
/// @brief 模型与点云导入

#include "DocumentPage.h"
#include "MainWindow.h"
#include "MainWindowImportCaptureRenderController.h"
#include "MainWindowSelectionService.h"
#include "MainWindow_p.h"
#include "RunInfoPage.h"
#include "WidgetRenderAccess.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFileImport.h"
#include "BackendTypeIds.h"
#include "CustomDeviceAssemblyDialog.h"
#include "CustomDeviceBackendData.h"
#include "DocumentImportFacade.h"
#include "FrameBackendData.h"
#include "MainWindowRobotHost.h"

#include <memory>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLatin1String>
#include <QLineEdit>
#include <QMessageBox>
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
	if (backendId.isEmpty() || !m_backendTree || !m_unitsTreeBinder)
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
	const QStringList filePaths =
		QFileDialog::getOpenFileNames(this, QStringLiteral("Open Model"), QString(), filter);
	if (filePaths.isEmpty())
	{
		return;
	}

	if (!renderWidgetFromPage(currentPage()))
	{
		QMessageBox::warning(this, QStringLiteral("Import Model"), QStringLiteral("No active 3D view."));
		return;
	}

	QString qualityProbePath;
	for (const QString& path : filePaths)
	{
		if (isMeshQualityPromptExtension(QFileInfo(path).suffix().toLower()))
		{
			qualityProbePath = path;
			break;
		}
	}
	if (!qualityProbePath.isEmpty())
	{
		const int quality = promptMeshImportQuality(*this, qualityProbePath);
		if (quality < 0)
		{
			return;
		}
	}

	int opened = 0;
	for (const QString& filePath : filePaths)
	{
		if (!registerBackendObject(filePath, QLatin1String(backend_type::kCatalogModel), false))
		{
			continue;
		}
		++opened;
		if (m_runInfoPage)
		{
			m_runInfoPage->appendInfo(QStringLiteral("Model opened: %1").arg(filePath));
		}
	}
	if (opened == 0 && !filePaths.isEmpty())
	{
		QMessageBox::warning(this, QStringLiteral("Import Model"), QStringLiteral("Failed to import selected models."));
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

	registerBackendObject(filePath, QLatin1String(backend_type::kCatalogPointCloud), true);
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
	nameEdit->setText(i18n(QLatin1String(backend_type::kCatalogCoordinateFrame), QStringLiteral("坐标系")));
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
		name = i18n(QLatin1String(backend_type::kCatalogCoordinateFrame), QStringLiteral("坐标系"));
	}

	auto frame = std::make_shared<FrameBackendData>();
	frame->setName(name.toStdString());
	frame->setPose(BackendVec3{xSpin->value(), ySpin->value(), zSpin->value()});
	frame->setRotation(BackendVec3{rxSpin->value(), rySpin->value(), rzSpin->value()});

	QString parentId;
	if (m_selectionState.hasBackendSelection())
	{
		const QString sel = m_selectionState.selectedBackendId();
		if (host->backend().contains(sel.toStdString()))
		{
			parentId = sel;
		}
	}

	QString err;
	if (!cloudsim::host::registerAdoptedFrameAndLoadScene(*host, frame, QLatin1String(backend_type::kCatalogCoordinateFrame),
														  parentId, false, &err))
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

void MainWindow::onCreateCustomDevice()
{
	openCustomDeviceAssemblyDialog(QString());
}

void MainWindow::onEditCustomDevice()
{
	cloudsim::host::DocumentHost* host = currentDocumentHost();
	if (!host)
	{
		QMessageBox::warning(
			this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
			i18n(QStringLiteral("No active document / 3D view."), QStringLiteral("没有活动文档或三维视图。")));
		return;
	}

	QStringList labels;
	QStringList ids;
	for (const auto& data : host->backend().listData())
	{
		if (!data || !backend_type::isCustomDeviceClassName(data->className()))
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		const QString name = QString::fromStdString(data->name().empty() ? data->id() : data->name());
		labels << QStringLiteral("%1 [%2]").arg(name, id);
		ids << id;
	}
	if (labels.isEmpty())
	{
		QMessageBox::information(
			this, i18n(QStringLiteral("Edit Custom Device"), QStringLiteral("编辑自定义设备")),
			i18n(QStringLiteral("No custom device to edit."), QStringLiteral("当前文档没有可编辑的自定义设备。")));
		return;
	}

	bool ok = false;
	const QString picked = QInputDialog::getItem(
		this,
		i18n(QStringLiteral("Edit Custom Device"), QStringLiteral("编辑自定义设备")),
		i18n(QStringLiteral("Select device:"), QStringLiteral("选择要修改的设备：")),
		labels,
		0,
		false,
		&ok);
	if (!ok || picked.isEmpty())
	{
		return;
	}
	const int idx = labels.indexOf(picked);
	if (idx < 0 || idx >= ids.size())
	{
		return;
	}
	openCustomDeviceAssemblyDialog(ids.at(idx));
}

void MainWindow::onExportCustomDeviceUrdf()
{
	(void)exportCustomDeviceUrdfInteractive(QString());
}

bool MainWindow::exportCustomDeviceUrdfInteractive(const QString& deviceBackendId)
{
	cloudsim::host::DocumentHost* host = currentDocumentHost();
	if (!host)
	{
		QMessageBox::warning(
			this, i18n(QStringLiteral("Export URDF"), QStringLiteral("导出 URDF")),
			i18n(QStringLiteral("No active document / 3D view."), QStringLiteral("没有活动文档或三维视图。")));
		return false;
	}

	QString deviceId = deviceBackendId.trimmed();
	if (deviceId.isEmpty())
	{
		QStringList labels;
		QStringList ids;
		for (const auto& data : host->backend().listData())
		{
			if (!data || !backend_type::isCustomDeviceClassName(data->className()))
			{
				continue;
			}
			const QString id = QString::fromStdString(data->id());
			const QString name = QString::fromStdString(data->name().empty() ? data->id() : data->name());
			labels << QStringLiteral("%1 [%2]").arg(name, id);
			ids << id;
		}
		if (labels.isEmpty())
		{
			QMessageBox::information(
				this, i18n(QStringLiteral("Export URDF"), QStringLiteral("导出 URDF")),
				i18n(QStringLiteral("No custom device to export."), QStringLiteral("当前文档没有可导出的自定义设备。")));
			return false;
		}
		bool ok = false;
		const QString picked = QInputDialog::getItem(
			this, i18n(QStringLiteral("Export URDF"), QStringLiteral("导出 URDF")),
			i18n(QStringLiteral("Select device:"), QStringLiteral("选择要导出的设备：")), labels, 0, false, &ok);
		if (!ok || picked.isEmpty())
		{
			return false;
		}
		const int idx = labels.indexOf(picked);
		if (idx < 0 || idx >= ids.size())
		{
			return false;
		}
		deviceId = ids.at(idx);
	}

	const QString parentDir = QFileDialog::getExistingDirectory(
		this, i18n(QStringLiteral("Select export folder"), QStringLiteral("选择导出目录")));
	if (parentDir.isEmpty())
	{
		return false;
	}

	QString urdfPath;
	QString packageRoot;
	QString err;
	if (!cloudsim::host::exportCustomDeviceUrdfPackage(*host, deviceId.toStdString(), parentDir, &urdfPath, &packageRoot,
													   &err))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Export URDF"), QStringLiteral("导出 URDF")), err);
		return false;
	}

	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(i18n(QStringLiteral("URDF exported: %1").arg(urdfPath),
									   QStringLiteral("已导出 URDF：%1").arg(urdfPath)));
	}
	QMessageBox::information(
		this, i18n(QStringLiteral("Export URDF"), QStringLiteral("导出 URDF")),
		i18n(QStringLiteral("Package written to:\n%1\n\nURDF:\n%2").arg(packageRoot, urdfPath),
			 QStringLiteral("已写出 ROS 包：\n%1\n\nURDF：\n%2").arg(packageRoot, urdfPath)));
	return true;
}

void MainWindow::openCustomDeviceAssemblyDialog(const QString& existingDeviceBackendId)
{
	if (!m_robotHost || !currentDocumentHost() || !currentPage() || !renderWidgetFromPage(currentPage()))
	{
		QMessageBox::warning(
			this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
			i18n(QStringLiteral("No active document / 3D view."), QStringLiteral("没有活动文档或三维视图。")));
		return;
	}

	CustomDeviceAssemblyDialog dialog(m_robotHost.get(), existingDeviceBackendId, this);
	dialog.show();
	QEventLoop loop;
	QObject::connect(&dialog, &QDialog::finished, &loop, &QEventLoop::quit);
	loop.exec();
}
