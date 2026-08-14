/// @file CustomDeviceAxisEditorWidget.cpp
/// @brief CustomDeviceAxisEditorWidget 实现

#include "CustomDeviceAxisEditorWidget.h"

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
#include <QMessageBox>
#include <QPushButton>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>

#include <algorithm>
#include <string>

namespace
{
constexpr double kDegToRad = 0.017453292519943295;
constexpr double kRadToDeg = 57.29577951308232;

QDoubleSpinBox* makeSpin(QWidget* parent, const double lo, const double hi, const double step = 1.0)
{
	auto* s = new QDoubleSpinBox(parent);
	s->setRange(lo, hi);
	s->setDecimals(3);
	s->setSingleStep(step);
	return s;
}

QString axisListLabel(const CustomDeviceAxisConfig& a)
{
	const QString name = QString::fromStdString(a.displayName.empty() ? a.jointName : a.displayName);
	const QString motion = a.motionType == CustomDeviceMotionType::Rotate ? QStringLiteral("R") : QStringLiteral("T");
	return QStringLiteral("%1 [%2]%3").arg(name, motion, a.enabled ? QString() : QStringLiteral(" [off]"));
}
} // namespace

CustomDeviceAxisEditorWidget::CustomDeviceAxisEditorWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 0, 0, 0);

	m_emptyHint = new QLabel(this);
	m_emptyHint->setWordWrap(true);
	root->addWidget(m_emptyHint);

	auto* listRow = new QHBoxLayout();
	m_list = new QListWidget(this);
	m_list->setMaximumHeight(120);
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
	m_motionCombo = new QComboBox(m_editorGroup);
	m_motionCombo->addItem(QStringLiteral("Translate"), static_cast<int>(CustomDeviceMotionType::Translate));
	m_motionCombo->addItem(QStringLiteral("Rotate"), static_cast<int>(CustomDeviceMotionType::Rotate));
	m_lowerSpin = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 10.0);
	m_upperSpin = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 10.0);
	m_homeSpin = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 10.0);
	for (int i = 0; i < 3; ++i)
	{
		m_axisSpin[i] = makeSpin(m_editorGroup, -1.0, 1.0, 0.1);
		m_originSpin[i] = makeSpin(m_editorGroup, -1.0e6, 1.0e6, 1.0);
		m_originSpin[i]->setSuffix(QStringLiteral(" mm"));
	}
	m_pickOriginBtn = new QPushButton(m_editorGroup);
	m_useNormalAsAxisCheck = new QCheckBox(m_editorGroup);
	m_useNormalAsAxisCheck->setChecked(true);

	m_form->addRow(QStringLiteral("Enabled"), m_enabledCheck);
	m_form->addRow(QStringLiteral("Name"), m_nameEdit);
	m_form->addRow(QStringLiteral("Motion"), m_motionCombo);
	m_form->addRow(QStringLiteral("Lower"), m_lowerSpin);
	m_form->addRow(QStringLiteral("Upper"), m_upperSpin);
	m_form->addRow(QStringLiteral("Home"), m_homeSpin);
	m_form->addRow(QStringLiteral("Axis X"), m_axisSpin[0]);
	m_form->addRow(QStringLiteral("Axis Y"), m_axisSpin[1]);
	m_form->addRow(QStringLiteral("Axis Z"), m_axisSpin[2]);
	m_form->addRow(QStringLiteral("Origin X"), m_originSpin[0]);
	m_form->addRow(QStringLiteral("Origin Y"), m_originSpin[1]);
	m_form->addRow(QStringLiteral("Origin Z"), m_originSpin[2]);
	m_form->addRow(QStringLiteral("Pick"), m_pickOriginBtn);
	m_form->addRow(QStringLiteral("Normal"), m_useNormalAsAxisCheck);
	root->addWidget(m_editorGroup);

	m_debounceTimer = new QTimer(this);
	m_debounceTimer->setSingleShot(true);
	m_debounceTimer->setInterval(80);

	connect(m_list, &QListWidget::currentRowChanged, this, &CustomDeviceAxisEditorWidget::onListSelectionChanged);
	connect(m_addBtn, &QPushButton::clicked, this, &CustomDeviceAxisEditorWidget::onAddAxis);
	connect(m_removeBtn, &QPushButton::clicked, this, &CustomDeviceAxisEditorWidget::onRemoveAxis);
	connect(m_enabledCheck, &QCheckBox::toggled, this, &CustomDeviceAxisEditorWidget::onFieldChanged);
	connect(m_nameEdit, &QLineEdit::textEdited, this, &CustomDeviceAxisEditorWidget::onFieldChanged);
	connect(m_motionCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&CustomDeviceAxisEditorWidget::onFieldChanged);
	connect(m_lowerSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&CustomDeviceAxisEditorWidget::onFieldChanged);
	connect(m_upperSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&CustomDeviceAxisEditorWidget::onFieldChanged);
	connect(m_homeSpin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
			&CustomDeviceAxisEditorWidget::onFieldChanged);
	for (int i = 0; i < 3; ++i)
	{
		connect(m_axisSpin[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&CustomDeviceAxisEditorWidget::onFieldChanged);
		connect(m_originSpin[i], QOverload<double>::of(&QDoubleSpinBox::valueChanged), this,
				&CustomDeviceAxisEditorWidget::onFieldChanged);
	}
	connect(m_pickOriginBtn, &QPushButton::clicked, this, &CustomDeviceAxisEditorWidget::pickOriginRequested);
	connect(m_debounceTimer, &QTimer::timeout, this, &CustomDeviceAxisEditorWidget::onChangedDebounce);

	CustomDeviceAxisConfigSet initial;
	initial.axes.push_back(makeDefaultCustomDeviceTranslateAxis());
	setAxes(initial);
	setUseChinese(true);
	m_editorGroup->setEnabled(false);
}

void CustomDeviceAxisEditorWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	m_addBtn->setText(chinese ? QStringLiteral("添加轴") : QStringLiteral("Add axis"));
	m_removeBtn->setText(chinese ? QStringLiteral("删除") : QStringLiteral("Remove"));
	m_editorGroup->setTitle(chinese ? QStringLiteral("轴参数") : QStringLiteral("Axis parameters"));
	m_enabledCheck->setText(chinese ? QStringLiteral("启用") : QStringLiteral("Enabled"));
	m_motionCombo->setItemText(0, chinese ? QStringLiteral("平移") : QStringLiteral("Translate"));
	m_motionCombo->setItemText(1, chinese ? QStringLiteral("旋转") : QStringLiteral("Rotate"));
	m_pickOriginBtn->setText(chinese ? QStringLiteral("拾取中心…") : QStringLiteral("Pick origin…"));
	m_useNormalAsAxisCheck->setText(chinese ? QStringLiteral("同时用法向作为旋转轴")
											: QStringLiteral("Also use face normal as axis"));
	if (m_form)
	{
		const QStringList labels =
			chinese ? QStringList{QStringLiteral("启用"),	 QStringLiteral("显示名"), QStringLiteral("运动类型"),
								  QStringLiteral("下限"),	 QStringLiteral("上限"),	 QStringLiteral("回零"),
								  QStringLiteral("轴 X"),	 QStringLiteral("轴 Y"),	 QStringLiteral("轴 Z"),
								  QStringLiteral("原点 X"), QStringLiteral("原点 Y"), QStringLiteral("原点 Z"),
								  QStringLiteral("拾取"),	 QStringLiteral("法向")}
					: QStringList{QStringLiteral("Enabled"), QStringLiteral("Name"),	  QStringLiteral("Motion"),
								  QStringLiteral("Lower"),	 QStringLiteral("Upper"),  QStringLiteral("Home"),
								  QStringLiteral("Axis X"),	 QStringLiteral("Axis Y"), QStringLiteral("Axis Z"),
								  QStringLiteral("Origin X"), QStringLiteral("Origin Y"), QStringLiteral("Origin Z"),
								  QStringLiteral("Pick"),	 QStringLiteral("Normal")};
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

void CustomDeviceAxisEditorWidget::setAxes(const CustomDeviceAxisConfigSet& axes)
{
	m_blockSignals = true;
	m_axes = axes;
	rebuildList();
	if (!m_axes.axes.empty())
	{
		const QSignalBlocker blocker(m_list);
		m_list->setCurrentRow(0);
	}
	loadFieldsFromSelection();
	m_blockSignals = false;
	refreshEmptyHint();
}

CustomDeviceAxisConfigSet CustomDeviceAxisEditorWidget::axes()
{
	saveFieldsToSelection();
	for (CustomDeviceAxisConfig& a : m_axes.axes)
	{
		normalizeCustomDeviceAxisConfig(a);
	}
	return m_axes;
}

int CustomDeviceAxisEditorWidget::currentAxisIndex() const
{
	return m_list ? m_list->currentRow() : -1;
}

bool CustomDeviceAxisEditorWidget::currentAxisIsRotate() const
{
	const int row = currentAxisIndex();
	if (row < 0 || row >= static_cast<int>(m_axes.axes.size()))
	{
		return false;
	}
	return m_axes.axes[static_cast<size_t>(row)].motionType == CustomDeviceMotionType::Rotate;
}

bool CustomDeviceAxisEditorWidget::useNormalAsAxis() const
{
	return m_useNormalAsAxisCheck && m_useNormalAsAxisCheck->isChecked();
}

void CustomDeviceAxisEditorWidget::applyPickedOriginLocalMm(const double x, const double y, const double z)
{
	const int row = currentAxisIndex();
	if (row < 0 || row >= static_cast<int>(m_axes.axes.size()))
	{
		return;
	}
	if (m_axes.axes[static_cast<size_t>(row)].motionType != CustomDeviceMotionType::Rotate)
	{
		return;
	}
	m_blockSignals = true;
	m_originSpin[0]->setValue(x);
	m_originSpin[1]->setValue(y);
	m_originSpin[2]->setValue(z);
	m_blockSignals = false;
	onFieldChanged();
}

void CustomDeviceAxisEditorWidget::applyPickedAxisDirection(const double x, const double y, const double z)
{
	m_blockSignals = true;
	m_axisSpin[0]->setValue(x);
	m_axisSpin[1]->setValue(y);
	m_axisSpin[2]->setValue(z);
	m_blockSignals = false;
	onFieldChanged();
}

void CustomDeviceAxisEditorWidget::rebuildList()
{
	const int row = m_list->currentRow();
	const QSignalBlocker blocker(m_list);
	m_list->clear();
	for (const CustomDeviceAxisConfig& a : m_axes.axes)
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

void CustomDeviceAxisEditorWidget::refreshMotionDependentUi()
{
	const bool rotate = m_motionCombo->currentData().toInt() == static_cast<int>(CustomDeviceMotionType::Rotate);
	for (int i = 0; i < 3; ++i)
	{
		m_originSpin[i]->setEnabled(rotate);
	}
	m_pickOriginBtn->setEnabled(rotate);
	m_useNormalAsAxisCheck->setEnabled(rotate);
	refreshLimitSuffixes();
}

void CustomDeviceAxisEditorWidget::refreshLimitSuffixes()
{
	const bool rotate = m_motionCombo->currentData().toInt() == static_cast<int>(CustomDeviceMotionType::Rotate);
	const QString suf = rotate ? QStringLiteral(" deg") : QStringLiteral(" mm");
	m_lowerSpin->setSuffix(suf);
	m_upperSpin->setSuffix(suf);
	m_homeSpin->setSuffix(suf);
	m_lowerSpin->setSingleStep(rotate ? 1.0 : 10.0);
	m_upperSpin->setSingleStep(rotate ? 1.0 : 10.0);
	m_homeSpin->setSingleStep(rotate ? 1.0 : 10.0);
}

void CustomDeviceAxisEditorWidget::loadFieldsFromSelection()
{
	const int row = m_list->currentRow();
	m_editorGroup->setEnabled(row >= 0 && row < static_cast<int>(m_axes.axes.size()));
	m_removeBtn->setEnabled(row >= 0);
	if (row < 0 || row >= static_cast<int>(m_axes.axes.size()))
	{
		return;
	}
	const CustomDeviceAxisConfig& a = m_axes.axes[static_cast<size_t>(row)];
	m_blockSignals = true;
	m_enabledCheck->setChecked(a.enabled);
	m_nameEdit->setText(QString::fromStdString(a.displayName));
	const int motionIdx =
		m_motionCombo->findData(static_cast<int>(a.motionType == CustomDeviceMotionType::Rotate
													 ? CustomDeviceMotionType::Rotate
													 : CustomDeviceMotionType::Translate));
	m_motionCombo->setCurrentIndex(motionIdx >= 0 ? motionIdx : 0);
	const bool rotate = a.motionType == CustomDeviceMotionType::Rotate;
	m_lowerSpin->setValue(rotate ? a.lower * kRadToDeg : a.lower);
	m_upperSpin->setValue(rotate ? a.upper * kRadToDeg : a.upper);
	m_homeSpin->setValue(rotate ? a.home * kRadToDeg : a.home);
	for (int i = 0; i < 3; ++i)
	{
		m_axisSpin[i]->setValue(a.axis[i]);
		m_originSpin[i]->setValue(a.originMm[i]);
	}
	m_blockSignals = false;
	refreshMotionDependentUi();
}

void CustomDeviceAxisEditorWidget::saveFieldsToSelection()
{
	const int row = m_list->currentRow();
	if (row < 0 || row >= static_cast<int>(m_axes.axes.size()))
	{
		return;
	}
	CustomDeviceAxisConfig& a = m_axes.axes[static_cast<size_t>(row)];
	a.enabled = m_enabledCheck->isChecked();
	a.displayName = m_nameEdit->text().trimmed().toStdString();
	a.motionType = m_motionCombo->currentData().toInt() == static_cast<int>(CustomDeviceMotionType::Rotate)
					   ? CustomDeviceMotionType::Rotate
					   : CustomDeviceMotionType::Translate;
	if (a.motionType == CustomDeviceMotionType::Rotate)
	{
		a.lower = m_lowerSpin->value() * kDegToRad;
		a.upper = m_upperSpin->value() * kDegToRad;
		a.home = m_homeSpin->value() * kDegToRad;
	}
	else
	{
		a.lower = m_lowerSpin->value();
		a.upper = m_upperSpin->value();
		a.home = m_homeSpin->value();
	}
	for (int i = 0; i < 3; ++i)
	{
		a.axis[i] = m_axisSpin[i]->value();
		a.originMm[i] = m_originSpin[i]->value();
	}
	normalizeCustomDeviceAxisConfig(a);
}

void CustomDeviceAxisEditorWidget::onListSelectionChanged()
{
	if (m_blockSignals)
	{
		return;
	}
	loadFieldsFromSelection();
}

void CustomDeviceAxisEditorWidget::onAddAxis()
{
	if (static_cast<int>(m_axes.axes.size()) >= kMaxAxes)
	{
		QMessageBox::information(this, m_useChinese ? QStringLiteral("自定义设备") : QStringLiteral("Custom Device"),
								 m_useChinese ? QStringLiteral("最多 %1 轴。").arg(kMaxAxes)
											  : QStringLiteral("At most %1 axes.").arg(kMaxAxes));
		return;
	}
	saveFieldsToSelection();
	CustomDeviceAxisConfig cfg = makeDefaultCustomDeviceTranslateAxis();
	cfg.displayName = m_useChinese ? "轴" + std::to_string(m_axes.axes.size() + 1)
								   : "Axis" + std::to_string(m_axes.axes.size() + 1);
	cfg.jointName = "device_joint_" + std::to_string(m_axes.axes.size() + 1);
	m_axes.axes.push_back(cfg);
	rebuildList();
	{
		const QSignalBlocker blocker(m_list);
		m_list->setCurrentRow(m_list->count() - 1);
	}
	loadFieldsFromSelection();
	scheduleChanged();
	refreshEmptyHint();
}

void CustomDeviceAxisEditorWidget::onRemoveAxis()
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
		const QSignalBlocker blocker(m_list);
		m_list->setCurrentRow(std::min(row, m_list->count() - 1));
	}
	loadFieldsFromSelection();
	scheduleChanged();
	refreshEmptyHint();
}

void CustomDeviceAxisEditorWidget::onFieldChanged()
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

void CustomDeviceAxisEditorWidget::scheduleChanged()
{
	m_debounceTimer->start();
}

void CustomDeviceAxisEditorWidget::onChangedDebounce()
{
	emit axesChanged();
}

void CustomDeviceAxisEditorWidget::refreshEmptyHint()
{
	const bool empty = m_axes.axes.empty();
	m_emptyHint->setVisible(empty);
	m_emptyHint->setText(m_useChinese ? QStringLiteral("添加至少一个运动轴（平移或旋转）。")
									  : QStringLiteral("Add at least one motion axis (translate or rotate)."));
}
