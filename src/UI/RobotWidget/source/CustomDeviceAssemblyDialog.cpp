/// @file CustomDeviceAssemblyDialog.cpp
/// @brief 自定义设备组装对话框

#include "CustomDeviceAssemblyDialog.h"

#include "BackendTypeIds.h"
#include "CustomDeviceAssemblyCanvasWidget.h"
#include "CustomDeviceAssemblyCommit.h"
#include "CustomDeviceBackendData.h"
#include "CustomDeviceKinematics.h"
#include "CustomDeviceRobotMountComponent.h"
#include "RobotExternalAxes.h"
#include "ICustomDeviceAssemblyHost.h"
#include "IRobotDocumentHost.h"

#include <QAbstractItemView>
#include <QComboBox>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFileDialog>
#include <QFileInfo>
#include <QFormLayout>
#include <QFrame>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSet>
#include <QSplitter>
#include <QVBoxLayout>

#include <cmath>
#include <unordered_set>

CustomDeviceAssemblyDialog::CustomDeviceAssemblyDialog(ICustomDeviceAssemblyHost* host,
													   const QString& existingDeviceBackendId, QWidget* parent)
	: QDialog(parent), m_host(host), m_editMode(!existingDeviceBackendId.trimmed().isEmpty())
{
	setObjectName(QStringLiteral("CustomDeviceAssemblyDialog"));
	setWindowTitle(m_editMode ? i18n(QStringLiteral("Edit Custom Device"), QStringLiteral("编辑自定义设备"))
							  : i18n(QStringLiteral("Assemble Custom Device"), QStringLiteral("自定义设备组装")));
	setWindowFlags(windowFlags() & ~Qt::WindowContextHelpButtonHint);
	setModal(false);
	setWindowModality(Qt::NonModal);
	resize(1040, 680);
	setStyleSheet(QStringLiteral(
		"#CustomDeviceAssemblyDialog {"
		"  background-color: #f4f5f7;"
		"}"
		"#CustomDeviceAssemblyDialog QFrame#assemblyHeader,"
		"#CustomDeviceAssemblyDialog QFrame#assemblyProps {"
		"  background-color: #ffffff;"
		"  border: 1px solid #dadcde;"
		"  border-radius: 8px;"
		"}"
		"#CustomDeviceAssemblyDialog QLabel#assemblyPropsTitle {"
		"  color: #1c1c1e;"
		"  font-weight: 600;"
		"  font-size: 13px;"
		"  padding-bottom: 4px;"
		"}"
		"#CustomDeviceAssemblyDialog QSplitter::handle {"
		"  background-color: transparent;"
		"  width: 8px;"
		"}"
		"#CustomDeviceAssemblyDialog QPushButton:checked {"
		"  background-color: #0066cc;"
		"  color: #ffffff;"
		"  border: 1px solid #0055aa;"
		"}"
		"#CustomDeviceAssemblyDialog CustomDeviceAssemblyCanvasWidget {"
		"  border: 1px solid #dadcde;"
		"  border-radius: 8px;"
		"  background-color: #eef1f5;"
		"}"));

	if (m_editMode && m_host && m_host->document())
	{
		m_device = std::dynamic_pointer_cast<CustomDeviceBackendData>(
			m_host->document()->findObject(existingDeviceBackendId.toStdString()));
		if (!m_device)
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Edit Custom Device"), QStringLiteral("编辑自定义设备")),
								 i18n(QStringLiteral("Custom device not found."), QStringLiteral("未找到自定义设备。")));
		}
	}

	auto* rootLayout = new QVBoxLayout(this);
	rootLayout->setContentsMargins(12, 12, 12, 12);
	rootLayout->setSpacing(10);

	auto* header = new QFrame(this);
	header->setObjectName(QStringLiteral("assemblyHeader"));
	auto* headerLayout = new QVBoxLayout(header);
	headerLayout->setContentsMargins(12, 10, 12, 10);
	headerLayout->setSpacing(8);

	m_nameEdit = new QLineEdit(header);
	if (m_editMode && m_device)
	{
		m_nameEdit->setText(
			QString::fromStdString(m_device->name().empty() ? m_device->id() : m_device->name()));
	}
	else
	{
		m_nameEdit->setText(i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")));
	}
	auto* topForm = new QFormLayout();
	topForm->setContentsMargins(0, 0, 0, 0);
	topForm->setHorizontalSpacing(10);
	topForm->setVerticalSpacing(6);
	topForm->addRow(i18n(QStringLiteral("Name"), QStringLiteral("名称")), m_nameEdit);
	headerLayout->addLayout(topForm);

	auto* toolRow = new QHBoxLayout();
	toolRow->setSpacing(6);
	auto* fromSceneBtn =
		new QPushButton(i18n(QStringLiteral("From scene…"), QStringLiteral("从场景选择…")), header);
	m_pickSolidBtn =
		new QPushButton(i18n(QStringLiteral("Pick solid in 3D"), QStringLiteral("3D 选择零件")), header);
	m_pickSolidBtn->setCheckable(true);
	m_pickSolidBtn->setToolTip(
		i18n(QStringLiteral("Click a face on an assembly in the 3D view to extract that Solid as a link."),
			 QStringLiteral("在视口中点击装配体的一个面，抽出该 Solid 作为独立连杆。")));
	auto* importFileBtn =
		new QPushButton(i18n(QStringLiteral("Import model…"), QStringLiteral("导入模型…")), header);
	m_connectBtn = new QPushButton(i18n(QStringLiteral("Connect"), QStringLiteral("连接")), header);
	m_connectBtn->setCheckable(true);
	auto* removeBtn = new QPushButton(i18n(QStringLiteral("Remove"), QStringLiteral("移除")), header);
	auto* setFixedBtn = new QPushButton(i18n(QStringLiteral("Set Fixed"), QStringLiteral("设为固定")), header);
	auto* exportUrdfBtn =
		new QPushButton(i18n(QStringLiteral("Export URDF…"), QStringLiteral("导出 URDF…")), header);
	for (QPushButton* b : {fromSceneBtn, m_pickSolidBtn, importFileBtn, m_connectBtn, setFixedBtn, exportUrdfBtn})
	{
		b->setProperty("btnRole", QStringLiteral("secondary"));
	}
	removeBtn->setProperty("btnRole", QStringLiteral("danger"));
	toolRow->addWidget(fromSceneBtn);
	toolRow->addWidget(m_pickSolidBtn);
	toolRow->addWidget(importFileBtn);
	toolRow->addWidget(m_connectBtn);
	toolRow->addWidget(removeBtn);
	toolRow->addWidget(setFixedBtn);
	toolRow->addWidget(exportUrdfBtn);
	toolRow->addStretch(1);
	headerLayout->addLayout(toolRow);
	rootLayout->addWidget(header);

	auto* splitter = new QSplitter(Qt::Horizontal, this);
	splitter->setChildrenCollapsible(false);
	splitter->setHandleWidth(8);
	m_canvas = new CustomDeviceAssemblyCanvasWidget(splitter);
	m_canvas->setUseChinese(m_host ? m_host->useChinese() : true);
	splitter->addWidget(m_canvas);

	m_props = new QFrame(splitter);
	m_props->setObjectName(QStringLiteral("assemblyProps"));
	auto* propsLayout = new QVBoxLayout(m_props);
	propsLayout->setContentsMargins(12, 12, 12, 12);
	propsLayout->setSpacing(8);
	auto* propsTitle =
		new QLabel(i18n(QStringLiteral("Joint Properties"), QStringLiteral("运动副属性")), m_props);
	propsTitle->setObjectName(QStringLiteral("assemblyPropsTitle"));
	propsLayout->addWidget(propsTitle);
	m_motionCombo = new QComboBox(m_props);
	m_motionCombo->addItem(i18n(QStringLiteral("Prismatic"), QStringLiteral("移动副")),
						   static_cast<int>(CustomDeviceMotionType::Translate));
	m_motionCombo->addItem(i18n(QStringLiteral("Revolute"), QStringLiteral("旋转副")),
						   static_cast<int>(CustomDeviceMotionType::Rotate));
	m_lowerSpin = new QDoubleSpinBox(m_props);
	m_upperSpin = new QDoubleSpinBox(m_props);
	m_homeSpin = new QDoubleSpinBox(m_props);
	for (QDoubleSpinBox* s : {m_lowerSpin, m_upperSpin, m_homeSpin})
	{
		s->setRange(-1e6, 1e6);
		s->setDecimals(4);
	}
	m_axisX = new QDoubleSpinBox(m_props);
	m_axisY = new QDoubleSpinBox(m_props);
	m_axisZ = new QDoubleSpinBox(m_props);
	for (QDoubleSpinBox* s : {m_axisX, m_axisY, m_axisZ})
	{
		s->setRange(-1e3, 1e3);
		s->setDecimals(4);
	}
	m_centerFrameCombo = new QComboBox(m_props);
	m_jointForm = new QFormLayout();
	m_jointForm->setContentsMargins(0, 0, 0, 0);
	m_jointForm->setHorizontalSpacing(10);
	m_jointForm->setVerticalSpacing(8);
	m_jointForm->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
	m_jointForm->addRow(i18n(QStringLiteral("Type"), QStringLiteral("类型")), m_motionCombo);
	m_jointForm->addRow(i18n(QStringLiteral("Lower"), QStringLiteral("下限")), m_lowerSpin);
	m_jointForm->addRow(i18n(QStringLiteral("Upper"), QStringLiteral("上限")), m_upperSpin);
	m_jointForm->addRow(i18n(QStringLiteral("Home"), QStringLiteral("Home")), m_homeSpin);
	m_jointForm->addRow(i18n(QStringLiteral("Axis X"), QStringLiteral("轴 X")), m_axisX);
	m_jointForm->addRow(i18n(QStringLiteral("Axis Y"), QStringLiteral("轴 Y")), m_axisY);
	m_jointForm->addRow(i18n(QStringLiteral("Axis Z"), QStringLiteral("轴 Z")), m_axisZ);
	m_jointForm->addRow(i18n(QStringLiteral("Rotation center"), QStringLiteral("旋转中心")), m_centerFrameCombo);
	propsLayout->addLayout(m_jointForm);
	propsLayout->addStretch(1);
	m_props->setMinimumWidth(260);
	m_props->setEnabled(false);
	splitter->addWidget(m_props);
	splitter->setStretchFactor(0, 3);
	splitter->setStretchFactor(1, 1);
	rootLayout->addWidget(splitter, 1);

	m_mountFrame = new QFrame(this);
	m_mountFrame->setObjectName(QStringLiteral("assemblyMount"));
	auto* mountLayout = new QVBoxLayout(m_mountFrame);
	mountLayout->setContentsMargins(12, 10, 12, 10);
	mountLayout->setSpacing(6);
	auto* mountTitle =
		new QLabel(i18n(QStringLiteral("Mount to Robot Flange"), QStringLiteral("安装到机器人法兰")), m_mountFrame);
	mountTitle->setObjectName(QStringLiteral("assemblyPropsTitle"));
	mountLayout->addWidget(mountTitle);
	auto* mountHint = new QLabel(
		i18n(QStringLiteral("Mount frame must be on the device root or a fixed link. On confirm, the device root follows the robot flange via Follow (install frame aligns with TCP)."),
			 QStringLiteral("安装坐标系须位于设备根或 fixed 连杆下。确认后设备根通过 Follow 跟随机器人法兰（安装坐标系与 TCP 对齐）。")),
		m_mountFrame);
	mountHint->setWordWrap(true);
	mountLayout->addWidget(mountHint);
	auto* mountForm = new QFormLayout();
	mountForm->setContentsMargins(0, 0, 0, 0);
	m_mountRobotCombo = new QComboBox(m_mountFrame);
	m_mountFlangeCombo = new QComboBox(m_mountFrame);
	m_mountFrameCombo = new QComboBox(m_mountFrame);
	mountForm->addRow(i18n(QStringLiteral("Robot"), QStringLiteral("机器人")), m_mountRobotCombo);
	mountForm->addRow(i18n(QStringLiteral("Flange link"), QStringLiteral("法兰连杆")), m_mountFlangeCombo);
	mountForm->addRow(i18n(QStringLiteral("Mount frame"), QStringLiteral("安装坐标系")), m_mountFrameCombo);
	mountLayout->addLayout(mountForm);
	auto* mountBtnRow = new QHBoxLayout();
	m_mountBtn = new QPushButton(i18n(QStringLiteral("Confirm mount"), QStringLiteral("确认挂载")), m_mountFrame);
	m_unmountBtn = new QPushButton(i18n(QStringLiteral("Unmount"), QStringLiteral("解除挂载")), m_mountFrame);
	m_mountBtn->setProperty("btnRole", QStringLiteral("primary"));
	m_unmountBtn->setProperty("btnRole", QStringLiteral("secondary"));
	mountBtnRow->addWidget(m_mountBtn);
	mountBtnRow->addWidget(m_unmountBtn);
	mountBtnRow->addStretch(1);
	mountLayout->addLayout(mountBtnRow);
	m_mountStatusLabel = new QLabel(m_mountFrame);
	m_mountStatusLabel->setWordWrap(true);
	mountLayout->addWidget(m_mountStatusLabel);
	rootLayout->addWidget(m_mountFrame);

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
	buttons->button(QDialogButtonBox::Ok)->setText(i18n(QStringLiteral("Apply"), QStringLiteral("应用")));
	buttons->button(QDialogButtonBox::Ok)->setProperty("btnRole", QStringLiteral("primary"));
	buttons->button(QDialogButtonBox::Cancel)->setProperty("btnRole", QStringLiteral("secondary"));
	rootLayout->addWidget(buttons);

	setRotationCenterVisible(false);
	refillCenterOptions();
	refillMountUi();
	if (m_editMode && m_device)
	{
		preloadEditGraph();
	}

	connect(m_canvas, &CustomDeviceAssemblyCanvasWidget::selectionChanged, this,
			&CustomDeviceAssemblyDialog::refreshJointProps);
	connect(m_canvas, &CustomDeviceAssemblyCanvasWidget::graphChanged, this,
			&CustomDeviceAssemblyDialog::refillCenterOptions);
	connect(m_motionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&CustomDeviceAssemblyDialog::pushJointProps);
	connect(m_lowerSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&CustomDeviceAssemblyDialog::pushJointProps);
	connect(m_upperSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&CustomDeviceAssemblyDialog::pushJointProps);
	connect(m_homeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&CustomDeviceAssemblyDialog::pushJointProps);
	connect(m_axisX, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&CustomDeviceAssemblyDialog::pushJointProps);
	connect(m_axisY, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&CustomDeviceAssemblyDialog::pushJointProps);
	connect(m_axisZ, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&CustomDeviceAssemblyDialog::pushJointProps);
	connect(m_centerFrameCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&CustomDeviceAssemblyDialog::pushJointProps);
	connect(m_connectBtn, &QPushButton::toggled, m_canvas, &CustomDeviceAssemblyCanvasWidget::setConnectionMode);
	connect(removeBtn, &QPushButton::clicked, m_canvas, &CustomDeviceAssemblyCanvasWidget::removeSelected);
	connect(setFixedBtn, &QPushButton::clicked, this, [this]() { m_canvas->setSelectedFixed(true); });
	connect(exportUrdfBtn, &QPushButton::clicked, this, &CustomDeviceAssemblyDialog::onExportUrdf);
	connect(fromSceneBtn, &QPushButton::clicked, this, &CustomDeviceAssemblyDialog::onFromScene);
	connect(m_pickSolidBtn, &QPushButton::toggled, this, &CustomDeviceAssemblyDialog::onPickSolidToggled);
	connect(importFileBtn, &QPushButton::clicked, this, &CustomDeviceAssemblyDialog::onImportModels);
	connect(m_mountBtn, &QPushButton::clicked, this, &CustomDeviceAssemblyDialog::onMountToRobot);
	connect(m_unmountBtn, &QPushButton::clicked, this, &CustomDeviceAssemblyDialog::onUnmountFromRobot);
	connect(m_mountRobotCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			[this](int) { refillMountUi(); });
	connect(buttons, &QDialogButtonBox::accepted, this, &CustomDeviceAssemblyDialog::onApplyAccepted);
	connect(buttons, &QDialogButtonBox::rejected, this, &QDialog::reject);
	connect(this, &QDialog::finished, this, [this]() {
		if (m_pickSolidBtn)
		{
			m_pickSolidBtn->setChecked(false);
		}
		if (m_host)
		{
			m_host->endPickSolidInView();
		}
	});
}

QString CustomDeviceAssemblyDialog::i18n(const QString& en, const QString& zh) const
{
	return m_host ? m_host->i18n(en, zh) : zh;
}

void CustomDeviceAssemblyDialog::setRotationCenterVisible(const bool visible)
{
	m_centerFrameCombo->setVisible(visible);
	if (QWidget* lab = m_jointForm->labelForField(m_centerFrameCombo))
	{
		lab->setVisible(visible);
	}
}

void CustomDeviceAssemblyDialog::refillCenterOptions()
{
	if (!m_host || !m_host->document())
	{
		return;
	}
	const QString keep = m_centerFrameCombo->currentData().toString();
	m_centerFrameCombo->blockSignals(true);
	m_centerFrameCombo->clear();
	m_centerFrameCombo->addItem(i18n(QStringLiteral("(None)"), QStringLiteral("（无）")), QString());
	QSet<QString> seenGeom;
	for (const CustomDeviceLink& L : m_canvas->links())
	{
		if (L.geometryBackendId.empty())
		{
			continue;
		}
		const QString gid = QString::fromStdString(L.geometryBackendId);
		if (seenGeom.contains(gid))
		{
			continue;
		}
		seenGeom.insert(gid);
		const QString title =
			QString::fromStdString(L.displayName.empty() ? L.geometryBackendId : L.displayName);
		m_centerFrameCombo->addItem(i18n(QStringLiteral("Model: %1 [%2]").arg(title, gid),
										 QStringLiteral("模型：%1 [%2]").arg(title, gid)),
									gid);
	}
	for (const auto& data : m_host->document()->listObjects())
	{
		if (!data || !backend_type::isCoordinateFrameClassName(data->className()))
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		const QString name = QString::fromStdString(data->name().empty() ? data->id() : data->name());
		m_centerFrameCombo->addItem(i18n(QStringLiteral("Frame: %1 [%2]").arg(name, id),
										 QStringLiteral("坐标系：%1 [%2]").arg(name, id)),
									id);
	}
	const int fi = m_centerFrameCombo->findData(keep);
	m_centerFrameCombo->setCurrentIndex(fi >= 0 ? fi : 0);
	m_centerFrameCombo->blockSignals(false);
}

void CustomDeviceAssemblyDialog::refreshJointProps()
{
	m_blockProps = true;
	refillCenterOptions();
	const QString jid = m_canvas->selectedJointId();
	m_props->setEnabled(!jid.isEmpty());
	if (jid.isEmpty())
	{
		setRotationCenterVisible(false);
		m_blockProps = false;
		return;
	}
	for (const CustomDeviceJoint& J : m_canvas->joints())
	{
		if (QString::fromStdString(J.id) != jid)
		{
			continue;
		}
		m_motionCombo->setCurrentIndex(J.motion.motionType == CustomDeviceMotionType::Rotate ? 1 : 0);
		m_lowerSpin->setValue(J.motion.lower);
		m_upperSpin->setValue(J.motion.upper);
		m_homeSpin->setValue(J.motion.home);
		m_axisX->setValue(J.motion.axis[0]);
		m_axisY->setValue(J.motion.axis[1]);
		m_axisZ->setValue(J.motion.axis[2]);
		const QString fid = QString::fromStdString(J.motion.motionCenterFrameBackendId);
		const int fi = m_centerFrameCombo->findData(fid);
		m_centerFrameCombo->setCurrentIndex(fi >= 0 ? fi : 0);
		setRotationCenterVisible(J.motion.motionType == CustomDeviceMotionType::Rotate);
		break;
	}
	m_blockProps = false;
}

void CustomDeviceAssemblyDialog::pushJointProps()
{
	if (m_blockProps || m_canvas->selectedJointId().isEmpty())
	{
		return;
	}
	CustomDeviceAxisConfig m = makeDefaultCustomDeviceTranslateAxis();
	if (static_cast<CustomDeviceMotionType>(m_motionCombo->currentData().toInt()) == CustomDeviceMotionType::Rotate)
	{
		m = makeDefaultCustomDeviceRotateAxis();
	}
	m.lower = m_lowerSpin->value();
	m.upper = m_upperSpin->value();
	m.home = m_homeSpin->value();
	m.axis[0] = m_axisX->value();
	m.axis[1] = m_axisY->value();
	m.axis[2] = m_axisZ->value();
	m.displayName = m_canvas->selectedJointId().toStdString();
	m.jointName = m.displayName;
	if (m.motionType == CustomDeviceMotionType::Rotate)
	{
		m.motionCenterFrameBackendId = m_centerFrameCombo->currentData().toString().toStdString();
		if (m.motionCenterFrameBackendId.empty())
		{
			m.originMm[0] = 0.0;
			m.originMm[1] = 0.0;
			m.originMm[2] = 0.0;
		}
		else if (m_host && m_host->document())
		{
			const QString jid = m_canvas->selectedJointId();
			for (const CustomDeviceJoint& J : m_canvas->joints())
			{
				if (QString::fromStdString(J.id) != jid)
				{
					continue;
				}
				for (const CustomDeviceLink& L : m_canvas->links())
				{
					if (L.id != J.parentLinkId || L.geometryBackendId.empty())
					{
						continue;
					}
					if (m_device && m_device->usesLinkJointGraph())
					{
						(void)CustomDeviceKinematics::bakeJointMotionOriginFromParentLink(
							*m_device, m, J.parentLinkId, &m_host->document()->backend());
					}
					else
					{
						(void)CustomDeviceKinematics::bakeJointMotionOriginFromParentGeometry(
							m, L.geometryBackendId, &m_host->document()->backend());
					}
					break;
				}
				break;
			}
		}
	}
	else
	{
		m.motionCenterFrameBackendId.clear();
	}
	normalizeCustomDeviceAxisConfig(m);
	m_canvas->updateSelectedJointMotion(m);
	setRotationCenterVisible(m.motionType == CustomDeviceMotionType::Rotate);
}

bool CustomDeviceAssemblyDialog::ensureDevice(QString* outErr)
{
	if (!m_host)
	{
		if (outErr)
		{
			*outErr = i18n(QStringLiteral("Host unavailable."), QStringLiteral("宿主不可用。"));
		}
		return false;
	}
	QString name = m_nameEdit->text().trimmed();
	if (name.isEmpty())
	{
		name = i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备"));
	}
	if (!m_device)
	{
		if (m_editMode)
		{
			if (outErr)
			{
				*outErr = i18n(QStringLiteral("Custom device not found."), QStringLiteral("未找到自定义设备。"));
			}
			return false;
		}
		m_device = std::make_shared<CustomDeviceBackendData>();
		m_device->setName(name.toStdString());
		QString err;
		if (!m_host->registerCustomDevice(m_device, &err))
		{
			if (outErr)
			{
				*outErr = err.isEmpty()
							  ? i18n(QStringLiteral("Failed to create device."), QStringLiteral("创建设备失败。"))
							  : err;
			}
			m_device.reset();
			return false;
		}
	}
	else
	{
		m_device->setName(name.toStdString());
	}
	return true;
}

bool CustomDeviceAssemblyDialog::attachChildId(const QString& childId, QString* outErr)
{
	if (!m_host || !m_host->document())
	{
		if (outErr)
		{
			*outErr = i18n(QStringLiteral("Host unavailable."), QStringLiteral("宿主不可用。"));
		}
		return false;
	}
	if (childId.isEmpty())
	{
		if (outErr)
		{
			*outErr = i18n(QStringLiteral("Empty component id."), QStringLiteral("组件 id 为空。"));
		}
		return false;
	}
	if (m_childRootIds.contains(childId))
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
	if (childId.toStdString() == m_device->id())
	{
		if (outErr)
		{
			*outErr = i18n(QStringLiteral("Cannot attach the device to itself."),
						   QStringLiteral("不能把设备挂到自身下。"));
		}
		return false;
	}
	QString err;
	BackendMat4 preAttachWorld = BackendMat4::identity();
	bool havePreAttachWorld = false;
	if (const auto geom = m_host->document()->findObject(childId.toStdString()))
	{
		if (geom->hasPoseProperty())
		{
			preAttachWorld = geom->worldMatrix(&m_host->document()->backend());
			havePreAttachWorld = true;
		}
	}
	if (!m_host->attachChildToCustomDevice(m_device->id(), childId.toStdString(), &err))
	{
		if (outErr)
		{
			*outErr = err.isEmpty()
						  ? i18n(QStringLiteral("Failed to attach component."), QStringLiteral("挂接组件失败。"))
						  : err;
		}
		return false;
	}
	m_childRootIds.append(childId);
	QString title = childId;
	if (const auto data = m_host->document()->findObject(childId.toStdString()))
	{
		title = QString::fromStdString(data->name());
	}
	const bool first = m_canvas->links().isEmpty();
	const QPointF pos(120.0 + m_canvas->links().size() * 200.0, 160.0);
	const QString linkId = m_canvas->addLinkBlock(title, childId, pos, first);
	m_device->captureBaseWorldW0FromCurrentWorld();
	if (havePreAttachWorld)
	{
		double w0[16];
		double invW0[16];
		double gw[16];
		double rest[16];
		for (int i = 0; i < 16; ++i)
		{
			w0[i] = m_device->baseWorldW0().v[i];
			gw[i] = preAttachWorld.v[i];
		}
		if (RobotExternal::mat4InvertRigidColumnMajor(w0, invW0))
		{
			RobotExternal::mat4MulColumnMajor16(invW0, gw, rest);
			(void)m_canvas->setLinkRestInDeviceW0(linkId, rest);
		}
	}
	(void)CustomDeviceKinematics::applyQ(*m_device, &m_host->document()->backend(), m_host->document()->poseSink());
	m_host->markFollowAttachmentDirty(QString::fromStdString(m_device->id()));
	m_host->runFollowSolveAndSync();
	m_host->refreshBackendTree();
	return true;
}

void CustomDeviceAssemblyDialog::preloadEditGraph()
{
	if (!m_device || !m_host || !m_host->document())
	{
		return;
	}
	if (m_device->links().empty())
	{
		return;
	}
	QVector<CustomDeviceLink> preloadLinks;
	QVector<CustomDeviceJoint> preloadJoints;
	for (const CustomDeviceLink& L : m_device->links())
	{
		preloadLinks.push_back(L);
		if (!L.geometryBackendId.empty())
		{
			m_childRootIds.append(QString::fromStdString(L.geometryBackendId));
		}
	}
	for (const CustomDeviceJoint& J : m_device->joints())
	{
		preloadJoints.push_back(J);
	}
	bool needLayout = !preloadLinks.isEmpty();
	for (const CustomDeviceLink& L : preloadLinks)
	{
		if (std::fabs(L.canvasX) > 1.0 || std::fabs(L.canvasY) > 1.0)
		{
			needLayout = false;
			break;
		}
	}
	if (needLayout)
	{
		for (int i = 0; i < preloadLinks.size(); ++i)
		{
			preloadLinks[i].canvasX = 120.0 + i * 200.0;
			preloadLinks[i].canvasY = 160.0;
		}
	}
	m_canvas->setGraph(preloadLinks, preloadJoints);
}

void CustomDeviceAssemblyDialog::onExportUrdf()
{
	if (!m_host)
	{
		return;
	}
	if (!m_device)
	{
		QMessageBox::information(
			this, i18n(QStringLiteral("Export URDF"), QStringLiteral("导出 URDF")),
			i18n(QStringLiteral("Create or open a device first (add a component)."),
				 QStringLiteral("请先添加组件以创建设备，或打开已有设备后再导出。")));
		return;
	}
	if (!m_device->usesLinkJointGraph())
	{
		QMessageBox::information(
			this, i18n(QStringLiteral("Export URDF"), QStringLiteral("导出 URDF")),
			m_canvas->joints().isEmpty()
				? i18n(QStringLiteral("Define links and joints first."), QStringLiteral("请先定义连杆与运动副。"))
				: i18n(QStringLiteral("Click Apply to save the assembly, then export URDF."),
					   QStringLiteral("请先点「应用」保存组装图，再导出 URDF。")));
		return;
	}
	(void)m_host->exportCustomDeviceUrdfInteractive(QString::fromStdString(m_device->id()));
}

void CustomDeviceAssemblyDialog::onFromScene()
{
	if (!m_host || !m_host->document())
	{
		return;
	}
	std::unordered_set<std::string> already;
	for (const QString& id : m_childRootIds)
	{
		already.insert(id.toStdString());
	}
	if (m_device)
	{
		already.insert(m_device->id());
	}
	QDialog pickDlg(this);
	pickDlg.setWindowTitle(i18n(QStringLiteral("Select Component"), QStringLiteral("选择组件")));
	pickDlg.setWindowFlags(pickDlg.windowFlags() & ~Qt::WindowContextHelpButtonHint);
	pickDlg.resize(420, 360);
	auto* pickLayout = new QVBoxLayout(&pickDlg);
	pickLayout->addWidget(new QLabel(
		i18n(QStringLiteral("Hold Ctrl/Shift to multi-select."), QStringLiteral("按住 Ctrl/Shift 可多选。")),
		&pickDlg));
	auto* list = new QListWidget(&pickDlg);
	list->setSelectionMode(QAbstractItemView::ExtendedSelection);
	for (const auto& data : m_host->document()->listObjects())
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
		auto* item =
			new QListWidgetItem(QStringLiteral("%1 [%2]").arg(QString::fromStdString(data->name()), id), list);
		item->setData(Qt::UserRole, id);
	}
	if (list->count() == 0)
	{
		QMessageBox::information(
			this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
			i18n(QStringLiteral("No available Mesh / STEP objects in the scene."),
				 QStringLiteral("场景中没有可挂接的网格 / STEP 对象。")));
		return;
	}
	pickLayout->addWidget(list, 1);
	auto* pickButtons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel, &pickDlg);
	pickLayout->addWidget(pickButtons);
	connect(pickButtons, &QDialogButtonBox::accepted, &pickDlg, &QDialog::accept);
	connect(pickButtons, &QDialogButtonBox::rejected, &pickDlg, &QDialog::reject);
	connect(list, &QListWidget::itemDoubleClicked, &pickDlg, &QDialog::accept);
	if (pickDlg.exec() != QDialog::Accepted)
	{
		return;
	}
	const QList<QListWidgetItem*> selected = list->selectedItems();
	if (selected.isEmpty())
	{
		return;
	}
	QStringList errors;
	for (QListWidgetItem* item : selected)
	{
		QString err;
		if (!attachChildId(item->data(Qt::UserRole).toString(), &err))
		{
			errors << err;
		}
	}
	if (!errors.isEmpty())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
							 errors.join(QLatin1Char('\n')));
	}
}

void CustomDeviceAssemblyDialog::onPickSolidToggled(const bool on)
{
	if (!m_host)
	{
		return;
	}
	if (!on)
	{
		m_host->endPickSolidInView();
		return;
	}
	QString err;
	if (!ensureDevice(&err))
	{
		m_pickSolidBtn->blockSignals(true);
		m_pickSolidBtn->setChecked(false);
		m_pickSolidBtn->blockSignals(false);
		QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")), err);
		return;
	}
	m_host->beginPickSolidInView([this](const QString& partId) {
		if (partId.isEmpty() || m_childRootIds.contains(partId))
		{
			return;
		}
		QString attachErr;
		if (!attachChildId(partId, &attachErr) && !attachErr.isEmpty())
		{
			QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
								 attachErr);
		}
	});
}

void CustomDeviceAssemblyDialog::onImportModels()
{
	if (!m_host)
	{
		return;
	}
	const QString filter = QStringLiteral(
		"Model Files (*.obj *.stl *.ply *.off *.dxf *.dae *.3ds *.fbx *.step *.stp *.igs *.iges);;All Files (*.*)");
	const QStringList paths = QFileDialog::getOpenFileNames(
		this, i18n(QStringLiteral("Select Model"), QStringLiteral("选择模型")), QString(), filter);
	if (paths.isEmpty())
	{
		return;
	}
	QString err;
	if (!ensureDevice(&err))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")), err);
		return;
	}
	QStringList importErrors;
	const QStringList roots = m_host->importModelsForAssembly(this, paths, &importErrors);
	QStringList errors = importErrors;
	int attached = 0;
	for (const QString& rootId : roots)
	{
		QString attachErr;
		if (!attachChildId(rootId, &attachErr))
		{
			errors << attachErr;
			continue;
		}
		++attached;
	}
	if (!errors.isEmpty() && (attached == 0 || !errors.isEmpty()))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
							 errors.join(QLatin1Char('\n')));
	}
}

void CustomDeviceAssemblyDialog::onApplyAccepted()
{
	if (!m_host || !m_host->document())
	{
		reject();
		return;
	}
	if (!m_device)
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
							 i18n(QStringLiteral("Add at least one component."), QStringLiteral("请至少添加一个组件。")));
		return;
	}
	const QVector<CustomDeviceLink> links = m_canvas->links();
	const QVector<CustomDeviceJoint> joints = m_canvas->joints();
	if (links.isEmpty())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
							 i18n(QStringLiteral("Add at least one Link block."), QStringLiteral("请至少添加一个 Link 块。")));
		return;
	}
	if (joints.isEmpty())
	{
		QMessageBox::warning(
			this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
			i18n(QStringLiteral("Connect Links to define at least one joint."),
				 QStringLiteral("请连接块以定义至少一个运动副。")));
		return;
	}

	std::vector<CustomDeviceLink> linkStd(links.begin(), links.end());
	std::vector<CustomDeviceJoint> jointStd(joints.begin(), joints.end());
	if (!CustomDeviceAssemblyCommit::commitGraph(*m_device, linkStd, jointStd, m_host->document()->backend(),
												 m_host->document()->poseSink()))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Custom Device"), QStringLiteral("自定义设备")),
							 i18n(QStringLiteral("Failed to apply assembly."), QStringLiteral("应用组装失败。")));
		return;
	}

	m_committedDeviceId = QString::fromStdString(m_device->id());
	m_host->onCustomDeviceAssemblyCommitted(m_committedDeviceId);
	m_host->markFollowAttachmentDirty(QString::fromStdString(m_device->id()));
	m_host->runFollowSolveAndSync();
	m_host->refreshBackendTree();
	m_host->focusBackendInTree(QString::fromStdString(m_device->id()));
	const QString name = QString::fromStdString(m_device->name());
	m_host->appendRunInfo(m_editMode
							  ? i18n(QStringLiteral("Custom device updated: %1").arg(name),
									 QStringLiteral("已更新自定义设备：%1").arg(name))
							  : i18n(QStringLiteral("Custom device assembled: %1").arg(name),
									 QStringLiteral("已组装自定义设备：%1").arg(name)));
	refillMountUi();
	updateMountControlsEnabled();
	accept();
}

void CustomDeviceAssemblyDialog::refillMountUi()
{
	if (!m_host || !m_mountRobotCombo || !m_mountFrameCombo)
	{
		return;
	}
	const QString deviceId = m_device ? QString::fromStdString(m_device->id()) : QString();
	m_mountRobotCombo->blockSignals(true);
	m_mountFrameCombo->blockSignals(true);
	const QString keepRobot = m_mountRobotCombo->currentData().toString();
	const QString keepFrame = m_mountFrameCombo->currentData().toString();
	m_mountRobotCombo->clear();
	const QVector<CustomDeviceMountRobotCandidate> robots = m_host->listMountRobotCandidates();
	for (const CustomDeviceMountRobotCandidate& r : robots)
	{
		const QString label = r.label.isEmpty() ? r.sceneBackendId : r.label;
		m_mountRobotCombo->addItem(label, r.sceneBackendId);
		m_mountRobotCombo->setItemData(m_mountRobotCombo->count() - 1, r.flangeLinkName, Qt::UserRole + 1);
		m_mountRobotCombo->setItemData(m_mountRobotCombo->count() - 1, r.flangeBackendId, Qt::UserRole + 2);
	}
	int robotIdx = m_mountRobotCombo->findData(keepRobot);
	if (robotIdx < 0)
	{
		robotIdx = 0;
	}
	m_mountRobotCombo->setCurrentIndex(robotIdx);

	m_mountFlangeCombo->clear();
	if (m_mountRobotCombo->count() > 0)
	{
		const int idx = m_mountRobotCombo->currentIndex();
		const QString flangeLink = m_mountRobotCombo->itemData(idx, Qt::UserRole + 1).toString();
		if (!flangeLink.isEmpty())
		{
			m_mountFlangeCombo->addItem(flangeLink, flangeLink);
		}
	}

	m_mountFrameCombo->clear();
	m_mountFrameCombo->addItem(i18n(QStringLiteral("(None)"), QStringLiteral("（无）")), QString());
	if (!deviceId.isEmpty())
	{
		const QVector<CustomDeviceMountFrameCandidate> frames = m_host->listMountFrameCandidates(deviceId);
		for (const CustomDeviceMountFrameCandidate& f : frames)
		{
			m_mountFrameCombo->addItem(f.displayName, f.backendId);
		}
	}
	int frameIdx = m_mountFrameCombo->findData(keepFrame);
	if (frameIdx < 0 && m_mountFrameCombo->count() > 1)
	{
		frameIdx = 1;
	}
	else if (frameIdx < 0)
	{
		frameIdx = 0;
	}
	m_mountFrameCombo->setCurrentIndex(frameIdx);
	m_mountRobotCombo->blockSignals(false);
	m_mountFrameCombo->blockSignals(false);

	if (m_mountStatusLabel)
	{
		if (!deviceId.isEmpty() && m_host->isDeviceMountedToRobot(deviceId))
		{
			m_mountStatusLabel->setText(i18n(QStringLiteral("Status: mounted to robot."),
										   QStringLiteral("状态：已挂载到机器人。")));
		}
		else
		{
			m_mountStatusLabel->setText(QString());
		}
	}
	updateMountControlsEnabled();
}

void CustomDeviceAssemblyDialog::updateMountControlsEnabled()
{
	const bool hasDevice = static_cast<bool>(m_device);
	const bool hasGraph = hasDevice && m_device->usesLinkJointGraph();
	const bool mounted = hasDevice && m_host && m_host->isDeviceMountedToRobot(QString::fromStdString(m_device->id()));
	if (m_mountFrame)
	{
		m_mountFrame->setEnabled(hasDevice);
	}
	if (m_mountBtn)
	{
		m_mountBtn->setEnabled(hasGraph && !mounted && m_mountRobotCombo && m_mountRobotCombo->count() > 0);
	}
	if (m_unmountBtn)
	{
		m_unmountBtn->setEnabled(mounted);
	}
}

void CustomDeviceAssemblyDialog::onMountToRobot()
{
	if (!m_host || !m_device)
	{
		return;
	}
	if (!m_device->usesLinkJointGraph())
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Mount"), QStringLiteral("安装")),
							 i18n(QStringLiteral("Apply assembly before mounting."),
								  QStringLiteral("请先应用组装（Link/Joint 图）。")));
		return;
	}
	if (m_mountRobotCombo->count() <= 0)
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Mount"), QStringLiteral("安装")),
							 i18n(QStringLiteral("No robot in scene."), QStringLiteral("场景中没有机器人。")));
		return;
	}
	const int rIdx = m_mountRobotCombo->currentIndex();
	const QString robotSceneId = m_mountRobotCombo->itemData(rIdx).toString();
	const QString flangeLink = m_mountFlangeCombo->count() > 0 ? m_mountFlangeCombo->currentData().toString()
															   : m_mountRobotCombo->itemData(rIdx, Qt::UserRole + 1).toString();
	const QString flangeBackendId = m_mountRobotCombo->itemData(rIdx, Qt::UserRole + 2).toString();
	const QString mountFrameId = m_mountFrameCombo->currentData().toString();
	const QString deviceId = QString::fromStdString(m_device->id());
	QString err;
	if (!m_host->mountDeviceToRobot(deviceId, robotSceneId, flangeLink, flangeBackendId, mountFrameId, &err))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Mount"), QStringLiteral("安装")), err);
		return;
	}
	refillMountUi();
	m_host->appendRunInfo(i18n(QStringLiteral("Device mounted to robot flange."),
							   QStringLiteral("设备已挂载到机器人法兰。")));
}

void CustomDeviceAssemblyDialog::onUnmountFromRobot()
{
	if (!m_host || !m_device)
	{
		return;
	}
	QString err;
	const QString deviceId = QString::fromStdString(m_device->id());
	if (!m_host->unmountDeviceFromRobot(deviceId, &err))
	{
		QMessageBox::warning(this, i18n(QStringLiteral("Unmount"), QStringLiteral("解除挂载")), err);
		return;
	}
	m_host->runFollowSolveAndSync();
	refillMountUi();
	m_host->appendRunInfo(i18n(QStringLiteral("Device unmounted from robot."),
							   QStringLiteral("设备已从机器人解除挂载。")));
}
