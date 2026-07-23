/// @file RobotExternalAxisSettingsWidget.cpp
/// @brief 外部轴配置页：一期最多一条地轨

#include "RobotExternalAxisSettingsWidget.h"

#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
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
} // namespace

RobotExternalAxisSettingsWidget::RobotExternalAxisSettingsWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);

	m_emptyHint = new QLabel(this);
	m_emptyHint->setWordWrap(true);
	root->addWidget(m_emptyHint);

	auto* listRow = new QHBoxLayout();
	m_list = new QListWidget(this);
	m_list->setMaximumHeight(100);
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
	m_jointCombo = new QComboBox(m_editorGroup);
	m_jointCombo->setEditable(true);
	m_lowerSpin = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 10.0);
	m_upperSpin = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 10.0);
	m_homeSpin = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 10.0);
	for (int i = 0; i < 3; ++i)
	{
		m_axisSpin[i] = makeSpin(m_editorGroup, -1.0, 1.0, 0.1);
	}
	m_form->addRow(QStringLiteral("Enabled"), m_enabledCheck);
	m_form->addRow(QStringLiteral("Joint"), m_jointCombo);
	m_form->addRow(QStringLiteral("Lower"), m_lowerSpin);
	m_form->addRow(QStringLiteral("Upper"), m_upperSpin);
	m_form->addRow(QStringLiteral("Home"), m_homeSpin);
	m_form->addRow(QStringLiteral("Axis X"), m_axisSpin[0]);
	m_form->addRow(QStringLiteral("Axis Y"), m_axisSpin[1]);
	m_form->addRow(QStringLiteral("Axis Z"), m_axisSpin[2]);
	root->addWidget(m_editorGroup);
	root->addStretch(1);

	m_debounceTimer = new QTimer(this);
	m_debounceTimer->setSingleShot(true);
	m_debounceTimer->setInterval(80);

	connect(m_list, &QListWidget::currentRowChanged, this, &RobotExternalAxisSettingsWidget::onListSelectionChanged);
	connect(m_addBtn, &QPushButton::clicked, this, &RobotExternalAxisSettingsWidget::onAddAxis);
	connect(m_removeBtn, &QPushButton::clicked, this, &RobotExternalAxisSettingsWidget::onRemoveAxis);
	connect(m_enabledCheck, &QCheckBox::toggled, this, &RobotExternalAxisSettingsWidget::onFieldChanged);
	connect(m_jointCombo, &QComboBox::currentTextChanged, this, &RobotExternalAxisSettingsWidget::onFieldChanged);
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
	}
	connect(m_debounceTimer, &QTimer::timeout, this, &RobotExternalAxisSettingsWidget::onChangedDebounce);

	setUseChinese(true);
	refreshEmptyHint();
	m_editorGroup->setEnabled(false);
}

void RobotExternalAxisSettingsWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	m_addBtn->setText(chinese ? QStringLiteral("添加地轨") : QStringLiteral("Add rail"));
	m_removeBtn->setText(chinese ? QStringLiteral("删除") : QStringLiteral("Remove"));
	m_editorGroup->setTitle(chinese ? QStringLiteral("地轨参数") : QStringLiteral("Rail parameters"));
	m_enabledCheck->setText(chinese ? QStringLiteral("启用联动求解") : QStringLiteral("Enable coordinated IK"));
	if (m_form)
	{
		const QStringList labels =
			chinese ? QStringList{QStringLiteral("启用"), QStringLiteral("关节名"), QStringLiteral("下限(mm)"),
								  QStringLiteral("上限(mm)"), QStringLiteral("回零(mm)"), QStringLiteral("轴向 X"),
								  QStringLiteral("轴向 Y"), QStringLiteral("轴向 Z")}
					: QStringList{QStringLiteral("Enabled"), QStringLiteral("Joint"), QStringLiteral("Lower(mm)"),
								  QStringLiteral("Upper(mm)"), QStringLiteral("Home(mm)"), QStringLiteral("Axis X"),
								  QStringLiteral("Axis Y"), QStringLiteral("Axis Z")};
		for (int i = 0; i < labels.size() && i < m_form->rowCount(); ++i)
		{
			if (QLabel* lab = qobject_cast<QLabel*>(m_form->itemAt(i, QFormLayout::LabelRole)->widget()))
			{
				lab->setText(labels[i]);
			}
		}
	}
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
		const QString label = QString::fromStdString(a.displayName.empty() ? a.jointName : a.displayName);
		auto* item = new QListWidgetItem(label + (a.enabled ? QString() : QStringLiteral(" [off]")));
		m_list->addItem(item);
	}
	if (row >= 0 && row < m_list->count())
	{
		m_list->setCurrentRow(row);
	}
	m_addBtn->setEnabled(m_axes.axes.size() < 1);
	m_removeBtn->setEnabled(m_list->currentRow() >= 0);
	m_editorGroup->setEnabled(m_list->currentRow() >= 0);
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
	const QString jn = QString::fromStdString(a.jointName);
	const int idx = m_jointCombo->findText(jn);
	if (idx >= 0)
	{
		m_jointCombo->setCurrentIndex(idx);
	}
	else
	{
		m_jointCombo->setEditText(jn);
	}
	m_lowerSpin->setValue(a.lower);
	m_upperSpin->setValue(a.upper);
	m_homeSpin->setValue(a.home);
	for (int i = 0; i < 3; ++i)
	{
		m_axisSpin[i]->setValue(a.axis[i]);
	}
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
	a.jointName = m_jointCombo->currentText().trimmed().toStdString();
	a.displayName = a.jointName.empty() ? "Rail" : a.jointName;
	a.kind = RobotExternal::RobotExternalAxisKind::LinearRail;
	a.isPrismatic = true;
	a.lower = m_lowerSpin->value();
	a.upper = m_upperSpin->value();
	a.home = m_homeSpin->value();
	for (int i = 0; i < 3; ++i)
	{
		a.axis[i] = m_axisSpin[i]->value();
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
	rebuildList();
	scheduleChanged();
}

void RobotExternalAxisSettingsWidget::onListSelectionChanged()
{
	loadFieldsFromSelection();
}

void RobotExternalAxisSettingsWidget::onAddAxis()
{
	if (m_axes.axes.size() >= 1)
	{
		return;
	}
	RobotExternal::RobotExternalAxisConfig cfg = RobotExternal::makeDefaultLinearRailConfig();
	if (!m_jointNames.isEmpty())
	{
		cfg.jointName = m_jointNames.front().toStdString();
		cfg.displayName = cfg.jointName;
	}
	m_axes.axes.push_back(std::move(cfg));
	rebuildList();
	m_list->setCurrentRow(0);
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
		m_emptyHint->setText(m_useChinese ? QStringLiteral("未配置外部轴：联动求解不会启用。点击「添加地轨」开始配置。")
										 : QStringLiteral("No external axis: coordinated IK stays off until you add a rail."));
		m_emptyHint->show();
	}
	else
	{
		m_emptyHint->hide();
	}
}
