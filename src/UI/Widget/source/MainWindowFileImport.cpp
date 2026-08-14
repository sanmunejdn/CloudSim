/// @file MainWindowFileImport.cpp
/// @brief MainWindowFileImport 实现

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
#include "BackendHierarchyFollow.h"
#include "BackendTypeIds.h"
#include "CustomDeviceAxisEditorWidget.h"
#include "CustomDeviceBackendData.h"
#include "CustomDeviceKinematics.h"
#include "DocumentImportFacade.h"
#include "FrameBackendData.h"
#include "IRobotBackendPoseSink.h"
#include "IRobotOsgViewHost.h"
#include "MainWindowRobotHost.h"
#include "OsgWidget.h"
#include "PickTypes.h"
#include "RobotAxisControlWidget.h"
#include "RobotSimulationController.h"

#include <cmath>
#include <memory>
#include <unordered_set>
#include <QComboBox>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QEventLoop>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLatin1String>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
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

	registerBackendObject(filePath, QLatin1String(backend_type::kCatalogModel), false);
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

	QString err;
	if (!cloudsim::host::registerAdoptedFrameAndLoadScene(*host, frame, QLatin1String(backend_type::kCatalogCoordinateFrame), QString(),
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

void MainWindow::onCreateCustomDevice()
{
	cloudsim::host::DocumentHost* host = currentDocumentHost();
	DocumentPage* page = currentPage();
	if (!host || !page || !renderWidgetFromPage(page))
	{
		QMessageBox::warning(
			this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
			i18n(QStringLiteral("No active document / 3D view."), QStringLiteral("没有活动文档或三维视图。")));
		return;
	}

	QDialog dialog(this);
	dialog.setWindowTitle(i18n(QStringLiteral("Create Custom Device"), QStringLiteral("新建自定义设备")));
	dialog.setModal(false);
	dialog.setWindowModality(Qt::NonModal);
	dialog.resize(460, 620);
	auto* layout = new QVBoxLayout(&dialog);
	auto* form = new QFormLayout();

	auto* nameEdit = new QLineEdit(&dialog);
	nameEdit->setText(i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")));

	auto* childList = new QListWidget(&dialog);
	childList->setMinimumHeight(96);
	auto* fromSceneBtn =
		new QPushButton(i18n(QStringLiteral("From scene…"), QStringLiteral("从场景选择…")), &dialog);
	auto* importFileBtn =
		new QPushButton(i18n(QStringLiteral("Import file…"), QStringLiteral("导入文件…")), &dialog);
	auto* removeChildBtn = new QPushButton(i18n(QStringLiteral("Remove"), QStringLiteral("移除")), &dialog);
	auto* childBtnRow = new QHBoxLayout;
	childBtnRow->addWidget(fromSceneBtn);
	childBtnRow->addWidget(importFileBtn);
	childBtnRow->addWidget(removeChildBtn);
	childBtnRow->addStretch(1);

	auto* axisEditor = new CustomDeviceAxisEditorWidget(&dialog);
	axisEditor->setUseChinese(m_useChinese);

	form->addRow(i18n(QStringLiteral("Name"), QStringLiteral("名称")), nameEdit);
	layout->addLayout(form);
	layout->addWidget(new QLabel(i18n(QStringLiteral("Components (Mesh / STEP)"), QStringLiteral("组件（网格 / STEP）")),
								 &dialog));
	layout->addWidget(childList);
	layout->addLayout(childBtnRow);
	layout->addWidget(axisEditor, 1);

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &dialog);
	layout->addWidget(buttons);

	struct SceneState
	{
		std::shared_ptr<CustomDeviceBackendData> device;
		QStringList childRootIds;
		bool picking = false;
	};
	auto state = std::make_shared<SceneState>();

	auto cleanupPick = [this, state]()
	{
		if (!state->picking)
		{
			return;
		}
		state->picking = false;
		if (m_robotHost)
		{
			m_robotHost->clearMeshPickCommittedHandler();
			if (IRobotOsgViewHost* osg = m_robotHost->osgView())
			{
				osg->setMeshFacePickMode(false);
				osg->setMeshPickScopeBackendId(std::string());
			}
		}
	};

	auto refreshChildListUi = [childList, state, host]()
	{
		childList->clear();
		for (const QString& id : state->childRootIds)
		{
			QString label = id;
			if (const auto data = host->backend().getData(id.toStdString()))
			{
				label = QStringLiteral("%1 [%2] (%3)")
							.arg(QString::fromStdString(data->name()), id,
								 QString::fromStdString(data->className()));
			}
			auto* item = new QListWidgetItem(label);
			item->setData(Qt::UserRole, id);
			childList->addItem(item);
		}
	};

	auto syncDeviceAfterChildrenChange = [this, host, page, state]()
	{
		if (!state->device)
		{
			return;
		}
		state->device->captureBaseWorldW0FromCurrentWorld();
		IRobotBackendPoseSink* sink = page->urdfImportScenePoseSink();
		(void)CustomDeviceKinematics::applyQ(*state->device, &host->backend(), sink);
		page->markFollowAttachmentDirtyFromBackendMove(QString::fromStdString(state->device->id()));
		if (m_robotHost)
		{
			m_robotHost->runFollowSolveAndSyncForCurrentDocument();
		}
		refreshBackendTree();
	};

	auto ensureDevice = [&](QString* outErr) -> bool
	{
		QString name = nameEdit->text().trimmed();
		if (name.isEmpty())
		{
			name = i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备"));
		}
		if (!state->device)
		{
			state->device = std::make_shared<CustomDeviceBackendData>();
			state->device->setName(name.toStdString());
			QString err;
			if (!cloudsim::host::registerAdoptedCustomDeviceAndLoadScene(
					*host, state->device, QLatin1String(backend_type::kCatalogCustomDevice), QString(), false, &err))
			{
				if (outErr)
				{
					*outErr = err.isEmpty() ? i18n(QStringLiteral("Failed to create device."),
												   QStringLiteral("创建设备失败。"))
											: err;
				}
				state->device.reset();
				return false;
			}
		}
		else
		{
			state->device->setName(name.toStdString());
		}
		return true;
	};

	auto attachChildId = [&](const QString& childId, QString* outErr) -> bool
	{
		if (childId.isEmpty())
		{
			if (outErr)
			{
				*outErr = i18n(QStringLiteral("Empty component id."), QStringLiteral("组件 id 为空。"));
			}
			return false;
		}
		if (state->childRootIds.contains(childId))
		{
			if (outErr)
			{
				*outErr = i18n(QStringLiteral("Component already attached."), QStringLiteral("该组件已挂接。"));
			}
			return false;
		}
		if (!ensureDevice(outErr))
		{
			return false;
		}
		if (childId.toStdString() == state->device->id())
		{
			if (outErr)
			{
				*outErr = i18n(QStringLiteral("Cannot attach the device to itself."),
							   QStringLiteral("不能把设备挂到自身下。"));
			}
			return false;
		}
		QString err;
		if (!cloudsim::host::attachBackendChildToCustomDevice(*host, state->device->id(), childId.toStdString(), &err))
		{
			if (outErr)
			{
				*outErr = err.isEmpty() ? i18n(QStringLiteral("Failed to attach component."),
											   QStringLiteral("挂接组件失败。"))
										: err;
			}
			return false;
		}
		state->childRootIds.append(childId);
		refreshChildListUi();
		syncDeviceAfterChildrenChange();
		return true;
	};

	auto ensureHasComponents = [&](QString* outErr) -> bool
	{
		if (!ensureDevice(outErr))
		{
			return false;
		}
		if (state->childRootIds.isEmpty())
		{
			if (outErr)
			{
				*outErr = i18n(QStringLiteral("Add at least one component from scene or file."),
							   QStringLiteral("请从场景选择或导入至少一个组件。"));
			}
			return false;
		}
		return true;
	};

	QObject::connect(fromSceneBtn, &QPushButton::clicked, &dialog, [&]()
	{
		std::unordered_set<std::string> already;
		for (const QString& id : state->childRootIds)
		{
			already.insert(id.toStdString());
		}
		if (state->device)
		{
			already.insert(state->device->id());
		}

		QDialog pickDlg(&dialog);
		pickDlg.setWindowTitle(i18n(QStringLiteral("Select Component"), QStringLiteral("选择组件")));
		auto* pickLayout = new QVBoxLayout(&pickDlg);
		pickLayout->addWidget(
			new QLabel(i18n(QStringLiteral("Choose a loaded Mesh / STEP object:"),
						   QStringLiteral("选择已加载的网格 / STEP 对象：")),
					   &pickDlg));
		auto* combo = new QComboBox(&pickDlg);
		for (const auto& data : host->backend().listData())
		{
			if (!data)
			{
				continue;
			}
			const std::string& cn = data->className();
			if (!backend_type::isMeshClassName(cn) && !backend_type::isBrepWorkpieceClassName(cn))
			{
				continue;
			}
			if (already.count(data->id()) > 0)
			{
				continue;
			}
			const QString id = QString::fromStdString(data->id());
			const QString label = QStringLiteral("%1 [%2] (%3)")
									  .arg(QString::fromStdString(data->name()), id,
										   QString::fromStdString(cn));
			combo->addItem(label, id);
		}
		if (combo->count() == 0)
		{
			QMessageBox::information(
				&dialog, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
				i18n(QStringLiteral("No available Mesh / STEP objects in the scene."),
					 QStringLiteral("场景中没有可挂接的网格 / STEP 对象。")));
			return;
		}
		pickLayout->addWidget(combo);
		auto* pickButtons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &pickDlg);
		pickLayout->addWidget(pickButtons);
		QObject::connect(pickButtons, &QDialogButtonBox::accepted, &pickDlg, &QDialog::accept);
		QObject::connect(pickButtons, &QDialogButtonBox::rejected, &pickDlg, &QDialog::reject);
		if (pickDlg.exec() != QDialog::Accepted)
		{
			return;
		}
		QString err;
		if (!attachChildId(combo->currentData().toString(), &err))
		{
			QMessageBox::warning(&dialog, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")), err);
			return;
		}
		if (m_runInfoPage)
		{
			m_runInfoPage->appendInfo(
				i18n(QStringLiteral("Component attached from scene."), QStringLiteral("已从场景挂接组件。")));
		}
	});

	QObject::connect(importFileBtn, &QPushButton::clicked, &dialog, [&]()
	{
		const QString filter = QStringLiteral(
			"Model Files (*.obj *.stl *.ply *.off *.dxf *.step *.stp *.igs *.iges);;All Files (*.*)");
		const QString path = QFileDialog::getOpenFileName(
			&dialog, i18n(QStringLiteral("Select Model"), QStringLiteral("选择模型")), QString(), filter);
		if (path.isEmpty())
		{
			return;
		}
		QString err;
		if (!ensureDevice(&err))
		{
			QMessageBox::warning(&dialog, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")), err);
			return;
		}
		cloudsim::core::ImportOptionsDto opt;
		opt.resetViewToHome = false;
		QString importErr;
		const cloudsim::host::ImportFileResult imported = cloudsim::host::importFileIntoDocument(
			*host, path, cloudsim::host::ImportFileKind::Mesh, opt, &importErr);
		if (!imported.ok || imported.rootBackendId.isEmpty())
		{
			QMessageBox::warning(
				&dialog, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
				importErr.isEmpty() ? i18n(QStringLiteral("Model import failed."), QStringLiteral("模型导入失败。"))
									: importErr);
			return;
		}
		if (!attachChildId(imported.rootBackendId, &err))
		{
			QMessageBox::warning(&dialog, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")), err);
			return;
		}
		if (m_runInfoPage)
		{
			m_runInfoPage->appendInfo(
				i18n(QStringLiteral("Component imported and attached."), QStringLiteral("已导入并挂接组件。")));
		}
	});

	QObject::connect(removeChildBtn, &QPushButton::clicked, &dialog, [&]()
	{
		QListWidgetItem* item = childList->currentItem();
		if (!item)
		{
			return;
		}
		const QString childId = item->data(Qt::UserRole).toString();
		if (childId.isEmpty())
		{
			return;
		}
		host->backend().detachAllParents(childId.toStdString());
		if (OsgWidget* osg = host->osgWidget())
		{
			osg->setBackendParent(childId.toStdString(), std::string());
		}
		cloudsim::host::applyHierarchyFollowBinding(*host, childId.toStdString(), std::string());
		state->childRootIds.removeAll(childId);
		refreshChildListUi();
		syncDeviceAfterChildrenChange();
	});

	QObject::connect(axisEditor, &CustomDeviceAxisEditorWidget::pickOriginRequested, &dialog, [&]()
	{
		if (!axisEditor->currentAxisIsRotate())
		{
			QMessageBox::information(&dialog, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
									 i18n(QStringLiteral("Pick origin is only for rotate axes."),
										  QStringLiteral("拾取中心仅用于旋转轴。")));
			return;
		}
		QString err;
		if (!ensureHasComponents(&err))
		{
			QMessageBox::warning(&dialog, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")), err);
			return;
		}
		if (!m_robotHost)
		{
			return;
		}
		IRobotOsgViewHost* osg = m_robotHost->osgView();
		if (!osg || !state->device)
		{
			return;
		}
		cleanupPick();
		state->picking = true;
		if (!state->childRootIds.isEmpty())
		{
			osg->setMeshPickScopeBackendId(state->childRootIds.front().toStdString());
		}
		osg->setMeshFacePickMode(true);
		m_robotHost->setMeshPickCommittedHandler([this, state, axisEditor, cleanupPick](const PickResult& pick,
																					   const PickKind kind)
		{
			if (!state->picking || kind != PickKind::MeshFace || !pick.hit || !state->device)
			{
				cleanupPick();
				return;
			}
			state->device->captureBaseWorldW0FromCurrentWorld();
			double local[3]{};
			if (!CustomDeviceKinematics::worldPointToDeviceLocalMm(
					state->device->baseWorldW0(), static_cast<double>(pick.worldPoint.x()),
					static_cast<double>(pick.worldPoint.y()), static_cast<double>(pick.worldPoint.z()), local))
			{
				cleanupPick();
				return;
			}
			axisEditor->applyPickedOriginLocalMm(local[0], local[1], local[2]);
			if (axisEditor->useNormalAsAxis())
			{
				double dir[3]{};
				if (CustomDeviceKinematics::worldDirectionToDeviceLocal(
						state->device->baseWorldW0(), static_cast<double>(pick.meshNormalWorld.x()),
						static_cast<double>(pick.meshNormalWorld.y()), static_cast<double>(pick.meshNormalWorld.z()),
						dir))
				{
					axisEditor->applyPickedAxisDirection(dir[0], dir[1], dir[2]);
				}
			}
			cleanupPick();
			if (m_runInfoPage)
			{
				m_runInfoPage->appendInfo(
					i18n(QStringLiteral("Rotation origin picked."), QStringLiteral("已拾取旋转中心。")));
			}
		});
		if (m_runInfoPage)
		{
			m_runInfoPage->appendInfo(
				i18n(QStringLiteral("Click a mesh face to set rotation origin."),
					 QStringLiteral("请在模型面上点击以设置旋转中心。")));
		}
	});

	QObject::connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
	QObject::connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);
	QObject::connect(&dialog, &QDialog::finished, &dialog, [&](int) { cleanupPick(); });

	dialog.show();
	QEventLoop loop;
	QObject::connect(&dialog, &QDialog::finished, &loop, &QEventLoop::quit);
	loop.exec();
	cleanupPick();
	if (dialog.result() != QDialog::Accepted)
	{
		return;
	}

	QString err;
	if (!ensureHasComponents(&err))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")), err);
		return;
	}
	if (!state->device)
	{
		return;
	}

	CustomDeviceAxisConfigSet set = axisEditor->axes();
	if (set.axes.empty())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
							 i18n(QStringLiteral("Add at least one axis."), QStringLiteral("请至少添加一个轴。")));
		return;
	}
	std::vector<double> homes;
	homes.reserve(set.axes.size());
	for (const CustomDeviceAxisConfig& a : set.axes)
	{
		homes.push_back(a.home);
	}
	state->device->setAxes(set);
	state->device->setQValues(homes);
	state->device->captureBaseWorldW0FromCurrentWorld();
	IRobotBackendPoseSink* sink = page->urdfImportScenePoseSink();
	(void)CustomDeviceKinematics::applyQ(*state->device, &host->backend(), sink);
	page->markFollowAttachmentDirtyFromBackendMove(QString::fromStdString(state->device->id()));
	if (m_robotHost)
	{
		m_robotHost->runFollowSolveAndSyncForCurrentDocument();
	}
	refreshBackendTree();
	focusBackendInTreeAfterImport(QString::fromStdString(state->device->id()));
	if (m_robotSimulation)
	{
		m_robotSimulation->refreshAxisControlTargets();
		if (RobotAxisControlWidget* axis = m_robotHost ? m_robotHost->robotAxisControlPage() : nullptr)
		{
			axis->selectControlTarget(AxisControlTargetKind::CustomDevice, QString::fromStdString(state->device->id()));
		}
	}
	const QString name = QString::fromStdString(state->device->name());
	if (m_runInfoPage)
	{
		m_runInfoPage->appendInfo(
			i18n(QStringLiteral("Custom device created: %1").arg(name), QStringLiteral("已创建自定义设备：%1").arg(name)));
	}
}

