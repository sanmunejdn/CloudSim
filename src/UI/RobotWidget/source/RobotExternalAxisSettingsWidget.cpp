/// @file RobotExternalAxisSettingsWidget.cpp
/// @brief 外部轴配置：多轴、运动类型、挂接与 backend 绑定

#include "RobotExternalAxisSettingsWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QLineEdit>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

namespace
{
QDoubleSpinBox* makeSpin(QWidget* parent, const double lo, const double hi, const double step = 1.0)
{
	auto* s = new QDoubleSpinBox(parent);
	s->setRange(lo, hi);
	s->setDecimals(3);
	s->setSingleStep(step);
	return s;
}

QString axisListLabel(const RobotExternal::RobotExternalAxisConfig& a)
{
	const QString name = QString::fromStdString(a.displayName.empty() ? a.jointName : a.displayName);
	const QString motion = a.motionType == RobotExternal::RobotExternalMotionType::Rotate ? QStringLiteral("R")
																						 : QStringLiteral("T");
	const QString att = a.attachment == RobotExternal::RobotExternalAttachment::Workpiece ? QStringLiteral("W")
																						  : QStringLiteral("B");
	return QStringLiteral("%1 [%2/%3]%4").arg(name, motion, att, a.enabled ? QString() : QStringLiteral(" [off]"));
}
} // namespace

RobotExternalAxisSettingsWidget::RobotExternalAxisSettingsWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);

	m_emptyHint = new QLabel(this);
	m_emptyHint->setWordWrap(true);
	root->addWidget(m_emptyHint);

	auto* listRow = new QHBoxLayout();
	m_list = new QListWidget(this);
	m_list->setMaximumHeight(140);
	listRow->addWidget(m_list, 1);
	auto* btnCol = new QVBoxLayout();
	m_addBtn = new QPushButton(this);
	m_removeBtn = new QPushButton(this);
	btnCol->addWidget(m_addBtn);
	btnCol->addWidget(m_removeBtn);
	btnCol->addStretch(1);
	listRow->addLayout(btnCol);
	root->addLayout(listRow);

	m_editorGroup = new QGroupBox(this);
	m_form = new QFormLayout(m_editorGroup);
	m_enabledCheck = new QCheckBox(m_editorGroup);
	m_nameEdit = new QLineEdit(m_editorGroup);
	m_jointCombo = new QComboBox(m_editorGroup);
	m_jointCombo->setEditable(true);
	m_motionCombo = new QComboBox(m_editorGroup);
	m_motionCombo->addItem(QStringLiteral("Translate"), static_cast<int>(RobotExternal::RobotExternalMotionType::Translate));
	m_motionCombo->addItem(QStringLiteral("Rotate"), static_cast<int>(RobotExternal::RobotExternalMotionType::Rotate));
	m_attachmentCombo = new QComboBox(m_editorGroup);
	m_attachmentCombo->addItem(QStringLiteral("RobotBase"),
							   static_cast<int>(RobotExternal::RobotExternalAttachment::RobotBase));
	m_attachmentCombo->addItem(QStringLiteral("Workpiece"),
							   static_cast<int>(RobotExternal::RobotExternalAttachment::Workpiece));
	m_backendCombo = new QComboBox(m_editorGroup);
	m_backendCombo->setEditable(false);
	m_workingFrameCombo = new QComboBox(m_editorGroup);
	// 仅下拉选场景 backend；空项=绑定根
	m_workingFrameCombo->setEditable(false);
	m_lowerSpin = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 10.0);
	m_upperSpin = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 10.0);
	m_homeSpin = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 10.0);
	for (int i = 0; i < 3; ++i)
	{
		m_axisSpin[i] = makeSpin(m_editorGroup, -1.0, 1.0, 0.1);
		m_originSpin[i] = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 1.0);
	}
	m_form->addRow(QStringLiteral("Enabled"), m_enabledCheck);
	m_form->addRow(QStringLiteral("Name"), m_nameEdit);
	m_form->addRow(QStringLiteral("Joint"), m_jointCombo);
	m_form->addRow(QStringLiteral("Motion"), m_motionCombo);
	m_form->addRow(QStringLiteral("Attachment"), m_attachmentCombo);
	m_form->addRow(QStringLiteral("Backend"), m_backendCombo);
	m_form->addRow(QStringLiteral("WorkingFrame"), m_workingFrameCombo);
	m_form->addRow(QStringLiteral("Lower"), m_lowerSpin);
	m_form->addRow(QStringLiteral("Upper"), m_upperSpin);
	m_form->addRow(QStringLiteral("Home"), m_homeSpin);
	m_form->addRow(QStringLiteral("Axis X"), m_axisSpin[0]);
	m_form->addRow(QStringLiteral("Axis Y"), m_axisSpin[1]);
	m_form->addRow(QStringLiteral("Axis Z"), m_axisSpin[2]);
	m_form->addRow(QStringLiteral("Origin X"), m_originSpin[0]);
	m_form->addRow(QStringLiteral("Origin Y"), m_originSpin[1]);
	m_form->addRow(QStringLiteral("Origin Z"), m_originSpin[2]);
	root->addWidget(m_editorGroup);
	root->addStretch(1);

	m_debounceTimer = new QTimer(this);
	m_debounceTimer->setSingleShot(true);
	m_debounceTimer->setInterval(80);

	connect(m_list, &QListWidget::currentRowChanged, this, &RobotExternalAxisSettingsWidget::onListSelectionChanged);
	connect(m_addBtn, &QPushButton::clicked, this, &RobotExternalAxisSettingsWidget::onAddAxis);
	connect(m_removeBtn, &QPushButton::clicked, this, &RobotExternalAxisSettingsWidget::onRemoveAxis);
	connect(m_enabledCheck, &QCheckBox::toggled, this, &RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_nameEdit, &QLineEdit::textEdited, this, &RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_jointCombo, &QComboBox::currentTextChanged, this, &RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_motionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_attachmentCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_backendCombo, &QComboBox::currentTextChanged, this, &RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_workingFrameCombo, &QComboBox::currentTextChanged, this, &RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_lowerSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_upperSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_homeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&RobotExternalAxisSettingsWidget::onFieldChanged);
	for (int i = 0; i < 3; ++i)
	{
		connect(m_axisSpin[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&RobotExternalAxisSettingsWidget::onFieldChanged);
		connect(m_originSpin[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&RobotExternalAxisSettingsWidget::onFieldChanged);
	}
	connect(m_debounceTimer, &QTimer::timeout, this, &RobotExternalAxisSettingsWidget::onChangedDebounce);

	setUseChinese(true);
	refreshEmptyHint();
	m_editorGroup->setEnabled(false);
}

void RobotExternalAxisSettingsWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	m_addBtn->setText(chinese ? QStringLiteral("添加轴") : QStringLiteral("Add axis"));
	m_removeBtn->setText(chinese ? QStringLiteral("删除") : QStringLiteral("Remove"));
	m_editorGroup->setTitle(chinese ? QStringLiteral("轴参数") : QStringLiteral("Axis parameters"));
	m_enabledCheck->setText(chinese ? QStringLiteral("启用联动求解") : QStringLiteral("Enable coordinated IK"));
	m_motionCombo->setItemText(0, chinese ? QStringLiteral("平移") : QStringLiteral("Translate"));
	m_motionCombo->setItemText(1, chinese ? QStringLiteral("旋转") : QStringLiteral("Rotate"));
	m_attachmentCombo->setItemText(0, chinese ? QStringLiteral("机器人基座") : QStringLiteral("RobotBase"));
	m_attachmentCombo->setItemText(1, chinese ? QStringLiteral("工件") : QStringLiteral("Workpiece"));
	m_backendCombo->setToolTip(chinese ? QStringLiteral("外轴驱动的工件场景节点（必选）")
									   : QStringLiteral("Scene backend driven by this workpiece axis"));
	m_workingFrameCombo->setToolTip(
		chinese ? QStringLiteral("示教/规划 TCP 相对的坐标系：默认「绑定根」；可选工件上其它 backend 作为偏置工作架")
				: QStringLiteral("Frame for relative TCP (REP). Empty = bound backend root; else offset subframe"));
	if (m_workingFrameCombo->count() > 0)
	{
		m_workingFrameCombo->setItemText(0, workingFrameBoundRootLabel());
	}
	if (m_form)
	{
		const QStringList labels =
			chinese ? QStringList{QStringLiteral("启用"),		 QStringLiteral("显示名"),	 QStringLiteral("关节名"),
								  QStringLiteral("运动类型"), QStringLiteral("作用对象"), QStringLiteral("绑定backend"),
								  QStringLiteral("工作架"),	 QStringLiteral("下限"),		 QStringLiteral("上限"),
								  QStringLiteral("回零"),		 QStringLiteral("轴 X"),		 QStringLiteral("轴 Y"),
								  QStringLiteral("轴 Z"),		 QStringLiteral("原点 X(mm)"), QStringLiteral("原点 Y(mm)"),
								  QStringLiteral("原点 Z(mm)")}
					: QStringList{QStringLiteral("Enabled"),		QStringLiteral("Name"),			QStringLiteral("Joint"),
								  QStringLiteral("Motion"),		QStringLiteral("Attachment"),	QStringLiteral("Backend"),
								  QStringLiteral("WorkingFrame"), QStringLiteral("Lower"),		QStringLiteral("Upper"),
								  QStringLiteral("Home"),		QStringLiteral("Axis X"),		QStringLiteral("Axis Y"),
								  QStringLiteral("Axis Z"),		QStringLiteral("Origin X"),		QStringLiteral("Origin Y"),
								  QStringLiteral("Origin Z")};
		for (int i = 0; i < labels.size() && i < m_form->rowCount(); ++i)
		{
			if (QLabel* lab = qobject_cast<QLabel*>(m_form->itemAt(i, QFormLayout::LabelRole)->widget()))
			{
				lab->setText(labels[i]);
			}
		}
	}
	refreshLimitSuffixes();
	refreshEmptyHint();
}

void RobotExternalAxisSettingsWidget::setJointNameOptions(const QStringList& jointNames)
{
	m_jointNames = jointNames;
	const QString cur = m_jointCombo->currentText();
	m_blockSignals = true;
	m_jointCombo->clear();
	m_jointCombo->addItems(m_jointNames);
	if (!cur.isEmpty())
	{
		const int idx = m_jointCombo->findText(cur);
		if (idx >= 0)
		{
			m_jointCombo->setCurrentIndex(idx);
		}
		else
		{
			m_jointCombo->setEditText(cur);
		}
	}
	m_blockSignals = false;
}

QString RobotExternalAxisSettingsWidget::workingFrameBoundRootLabel() const
{
	return m_useChinese ? QStringLiteral("（绑定根）") : QStringLiteral("(bound root)");
}

void RobotExternalAxisSettingsWidget::rebuildWorkingFrameOptions(const QString& selectedFrameId)
{
	m_workingFrameCombo->clear();
	m_workingFrameCombo->addItem(workingFrameBoundRootLabel(), QString());
	for (const QString& id : m_backendIds)
	{
		m_workingFrameCombo->addItem(id, id);
	}
	if (!selectedFrameId.isEmpty() && m_workingFrameCombo->findData(selectedFrameId) < 0)
	{
		m_workingFrameCombo->addItem(selectedFrameId, selectedFrameId);
	}
	const int idx = selectedFrameId.isEmpty() ? 0 : m_workingFrameCombo->findData(selectedFrameId);
	m_workingFrameCombo->setCurrentIndex(idx >= 0 ? idx : 0);
}

void RobotExternalAxisSettingsWidget::setBackendIdOptions(const QStringList& backendIds)
{
	m_backendIds = backendIds;
	const QString cur = m_backendCombo->currentData().toString().isEmpty() ? m_backendCombo->currentText()
																		  : m_backendCombo->currentData().toString();
	const QString curWork = m_workingFrameCombo->currentData().toString();
	m_blockSignals = true;
	m_backendCombo->clear();
	for (const QString& id : m_backendIds)
	{
		m_backendCombo->addItem(id, id);
	}
	if (!cur.isEmpty() && m_backendCombo->findData(cur) < 0)
	{
		m_backendCombo->addItem(cur, cur);
	}
	const int bIdx = cur.isEmpty() ? -1 : m_backendCombo->findData(cur);
	if (bIdx >= 0)
	{
		m_backendCombo->setCurrentIndex(bIdx);
	}
	else if (m_backendCombo->count() > 0)
	{
		m_backendCombo->setCurrentIndex(0);
	}
	rebuildWorkingFrameOptions(curWork);
	m_blockSignals = false;
}

void RobotExternalAxisSettingsWidget::setExternalAxes(const RobotExternal::RobotExternalAxisConfigSet& axes)
{
	m_blockSignals = true;
	m_axes = axes;
	rebuildList();
	if (!m_axes.axes.empty())
	{
		m_list->setCurrentRow(0);
	}
	loadFieldsFromSelection();
	m_blockSignals = false;
	refreshEmptyHint();
}

RobotExternal::RobotExternalAxisConfigSet RobotExternalAxisSettingsWidget::externalAxes()
{
	saveFieldsToSelection();
	for (RobotExternal::RobotExternalAxisConfig& a : m_axes.axes)
	{
		RobotExternal::normalizeExternalAxisConfig(a);
	}
	return m_axes;
}

void RobotExternalAxisSettingsWidget::rebuildList()
{
	const int row = m_list->currentRow();
	m_list->clear();
	for (const RobotExternal::RobotExternalAxisConfig& a : m_axes.axes)
	{
		m_list->addItem(new QListWidgetItem(axisListLabel(a)));
	}
	if (row >= 0 && row < m_list->count())
	{
		m_list->setCurrentRow(row);
	}
	m_removeBtn->setEnabled(m_list->currentRow() >= 0);
	m_editorGroup->setEnabled(m_list->currentRow() >= 0);
}

void RobotExternalAxisSettingsWidget::refreshMotionDependentUi()
{
	const bool rotate = m_motionCombo->currentData().toInt() ==
						static_cast<int>(RobotExternal::RobotExternalMotionType::Rotate);
	const bool workpiece = m_attachmentCombo->currentData().toInt() ==
						   static_cast<int>(RobotExternal::RobotExternalAttachment::Workpiece);
	for (int i = 0; i < 3; ++i)
	{
		m_originSpin[i]->setEnabled(rotate);
	}
	m_backendCombo->setEnabled(workpiece);
	m_workingFrameCombo->setEnabled(workpiece);
	refreshLimitSuffixes();
}

void RobotExternalAxisSettingsWidget::refreshLimitSuffixes()
{
	const bool rotate = m_motionCombo->currentData().toInt() ==
						static_cast<int>(RobotExternal::RobotExternalMotionType::Rotate);
	const QString suf = rotate ? (m_useChinese ? QStringLiteral(" rad") : QStringLiteral(" rad"))
							   : (m_useChinese ? QStringLiteral(" mm") : QStringLiteral(" mm"));
	m_lowerSpin->setSuffix(suf);
	m_upperSpin->setSuffix(suf);
	m_homeSpin->setSuffix(suf);
	m_lowerSpin->setSingleStep(rotate ? 0.01 : 10.0);
	m_upperSpin->setSingleStep(rotate ? 0.01 : 10.0);
	m_homeSpin->setSingleStep(rotate ? 0.01 : 10.0);
}

void RobotExternalAxisSettingsWidget::loadFieldsFromSelection()
{
	const int row = m_list->currentRow();
	m_editorGroup->setEnabled(row >= 0 && row < static_cast<int>(m_axes.axes.size()));
	m_removeBtn->setEnabled(row >= 0);
	if (row < 0 || row >= static_cast<int>(m_axes.axes.size()))
	{
		return;
	}
	const RobotExternal::RobotExternalAxisConfig& a = m_axes.axes[static_cast<size_t>(row)];
	m_blockSignals = true;
	m_enabledCheck->setChecked(a.enabled);
	m_nameEdit->setText(QString::fromStdString(a.displayName));
	const QString jn = QString::fromStdString(a.jointName);
	const int jIdx = m_jointCombo->findText(jn);
	if (jIdx >= 0)
	{
		m_jointCombo->setCurrentIndex(jIdx);
	}
	else
	{
		m_jointCombo->setEditText(jn);
	}
	m_motionCombo->setCurrentIndex(a.motionType == RobotExternal::RobotExternalMotionType::Rotate ? 1 : 0);
	m_attachmentCombo->setCurrentIndex(a.attachment == RobotExternal::RobotExternalAttachment::Workpiece ? 1 : 0);
	const QString be = QString::fromStdString(a.boundBackendId);
	if (!be.isEmpty() && m_backendCombo->findData(be) < 0)
	{
		m_backendCombo->addItem(be, be);
	}
	const int bIdx = be.isEmpty() ? -1 : m_backendCombo->findData(be);
	if (bIdx >= 0)
	{
		m_backendCombo->setCurrentIndex(bIdx);
	}
	else if (m_backendCombo->count() > 0)
	{
		m_backendCombo->setCurrentIndex(0);
	}
	rebuildWorkingFrameOptions(QString::fromStdString(a.workingFrameId));
	m_lowerSpin->setValue(a.lower);
	m_upperSpin->setValue(a.upper);
	m_homeSpin->setValue(a.home);
	for (int i = 0; i < 3; ++i)
	{
		m_axisSpin[i]->setValue(a.axis[i]);
		m_originSpin[i]->setValue(a.originMm[i]);
	}
	refreshMotionDependentUi();
	m_blockSignals = false;
}

void RobotExternalAxisSettingsWidget::saveFieldsToSelection()
{
	const int row = m_list->currentRow();
	if (row < 0 || row >= static_cast<int>(m_axes.axes.size()))
	{
		return;
	}
	RobotExternal::RobotExternalAxisConfig& a = m_axes.axes[static_cast<size_t>(row)];
	a.enabled = m_enabledCheck->isChecked();
	a.displayName = m_nameEdit->text().trimmed().toStdString();
	a.jointName = m_jointCombo->currentText().trimmed().toStdString();
	if (a.displayName.empty())
	{
		a.displayName = a.jointName.empty() ? "ExtAxis" : a.jointName;
	}
	a.motionType = static_cast<RobotExternal::RobotExternalMotionType>(m_motionCombo->currentData().toInt());
	a.attachment = static_cast<RobotExternal::RobotExternalAttachment>(m_attachmentCombo->currentData().toInt());
	a.boundBackendId = m_backendCombo->currentData().toString().trimmed().toStdString();
	if (a.boundBackendId.empty())
	{
		a.boundBackendId = m_backendCombo->currentText().trimmed().toStdString();
	}
	a.workingFrameId = m_workingFrameCombo->currentData().toString().trimmed().toStdString();
	a.lower = m_lowerSpin->value();
	a.upper = m_upperSpin->value();
	a.home = m_homeSpin->value();
	for (int i = 0; i < 3; ++i)
	{
		a.axis[i] = m_axisSpin[i]->value();
		a.originMm[i] = m_originSpin[i]->value();
	}
	RobotExternal::normalizeExternalAxisConfig(a);
}

void RobotExternalAxisSettingsWidget::scheduleChanged()
{
	if (m_blockSignals)
	{
		return;
	}
	m_debounceTimer->start();
}

void RobotExternalAxisSettingsWidget::onFieldChanged()
{
	if (m_blockSignals)
	{
		return;
	}
	saveFieldsToSelection();
	refreshMotionDependentUi();
	rebuildList();
	scheduleChanged();
}

void RobotExternalAxisSettingsWidget::onListSelectionChanged()
{
	loadFieldsFromSelection();
}

void RobotExternalAxisSettingsWidget::onAddAxis()
{
	RobotExternal::RobotExternalAxisConfig cfg = RobotExternal::makeDefaultLinearRailConfig();
	cfg.displayName = "ExtAxis" + std::to_string(m_axes.axes.size() + 1);
	if (!m_jointNames.isEmpty())
	{
		cfg.jointName = m_jointNames.front().toStdString();
	}
	m_axes.axes.push_back(std::move(cfg));
	rebuildList();
	m_list->setCurrentRow(static_cast<int>(m_axes.axes.size()) - 1);
	loadFieldsFromSelection();
	scheduleChanged();
	refreshEmptyHint();
}

void RobotExternalAxisSettingsWidget::onRemoveAxis()
{
	const int row = m_list->currentRow();
	if (row < 0 || row >= static_cast<int>(m_axes.axes.size()))
	{
		return;
	}
	m_axes.axes.erase(m_axes.axes.begin() + row);
	rebuildList();
	if (!m_axes.axes.empty())
	{
		m_list->setCurrentRow(0);
	}
	loadFieldsFromSelection();
	scheduleChanged();
	refreshEmptyHint();
}

void RobotExternalAxisSettingsWidget::onChangedDebounce()
{
	emit externalAxesChanged();
}

void RobotExternalAxisSettingsWidget::refreshEmptyHint()
{
	if (!m_emptyHint)
	{
		return;
	}
	if (m_axes.axes.empty())
	{
		m_emptyHint->setText(m_useChinese
								 ? QStringLiteral("未配置外部轴：联动求解不会启用。可添加平移/旋转轴，挂接机器人或工件。")
								 : QStringLiteral("No external axis: add Translate/Rotate axes on RobotBase or Workpiece."));
		m_emptyHint->show();
	}
	else
	{
		m_emptyHint->hide();
	}
}
