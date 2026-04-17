#include "SimulationCommandWidget.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>

#include <QPushButton>

#include <QVBoxLayout>

namespace
{

QString cmdLabel(const RobotSimulationCommand& c, const QStringList& jointNames, bool zh)
{
	const QString jn = (c.jointIndex >= 0 && c.jointIndex < jointNames.size()) ? jointNames[c.jointIndex]
																				 : QStringLiteral("?");
	if (zh)
	{
		return QStringLiteral("%1  %2° / %3 s").arg(jn).arg(c.angleDeg, 0, 'f', 1).arg(c.durationSec, 0, 'f', 1);
	}
	return QStringLiteral("%1  %2 deg / %3 s").arg(jn).arg(c.angleDeg, 0, 'f', 1).arg(c.durationSec, 0, 'f', 1);
}

} // namespace

SimulationCommandWidget::SimulationCommandWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(6);

	auto* hint = new QLabel(QStringLiteral("Add joint rotation segments for the imported robot (URDF)."));
	hint->setWordWrap(true);
	root->addWidget(hint);
	m_hintLabel = hint;

	auto* form = new QHBoxLayout;
	m_jointCombo = new QComboBox(this);
	form->addWidget(m_jointCombo);

	m_angleSpin = new QDoubleSpinBox(this);
	m_angleSpin->setRange(-360.0, 720.0);
	m_angleSpin->setDecimals(1);
	m_angleSpin->setValue(45.0);
	m_angleSpin->setSuffix(QStringLiteral(" deg"));
	form->addWidget(m_angleSpin);

	m_durationSpin = new QDoubleSpinBox(this);
	m_durationSpin->setRange(0.05, 120.0);
	m_durationSpin->setDecimals(2);
	m_durationSpin->setValue(2.0);
	m_durationSpin->setSuffix(QStringLiteral(" s"));
	form->addWidget(m_durationSpin);

	root->addLayout(form);

	auto* rowBtns = new QHBoxLayout;
	m_addBtn = new QPushButton(QStringLiteral("Add"));
	m_removeBtn = new QPushButton(QStringLiteral("Remove"));
	m_clearBtn = new QPushButton(QStringLiteral("Clear"));
	rowBtns->addWidget(m_addBtn);
	rowBtns->addWidget(m_removeBtn);
	rowBtns->addWidget(m_clearBtn);
	root->addLayout(rowBtns);

	m_list = new QListWidget(this);
	m_list->setMinimumHeight(120);
	root->addWidget(m_list, 1);

	auto* rowRun = new QHBoxLayout;
	m_runBtn = new QPushButton(QStringLiteral("Run"));
	m_stopBtn = new QPushButton(QStringLiteral("Stop"));
	m_stopBtn->setEnabled(false);
	rowRun->addWidget(m_runBtn);
	rowRun->addWidget(m_stopBtn);
	root->addLayout(rowRun);

	connect(m_addBtn, &QPushButton::clicked, this, &SimulationCommandWidget::onAddClicked);
	connect(m_removeBtn, &QPushButton::clicked, this, &SimulationCommandWidget::onRemoveClicked);
	connect(m_clearBtn, &QPushButton::clicked, this, &SimulationCommandWidget::onClearClicked);
	connect(m_runBtn, &QPushButton::clicked, this, &SimulationCommandWidget::runRequested);
	connect(m_stopBtn, &QPushButton::clicked, this, &SimulationCommandWidget::stopRequested);

	updateRunStopButtons();
}

void SimulationCommandWidget::updateRunStopButtons()
{
	const bool canRun = !m_jointNames.isEmpty() && !m_commands.isEmpty();
	m_runBtn->setEnabled(!m_simulationRunning && canRun);
	m_stopBtn->setEnabled(m_simulationRunning);
}

void SimulationCommandWidget::setRevoluteJointNames(const QStringList& names)
{
	m_jointNames = names;
	m_jointCombo->clear();
	for (const QString& n : names)
	{
		m_jointCombo->addItem(n);
	}
	const bool ok = !names.isEmpty();
	m_jointCombo->setEnabled(ok);
	m_angleSpin->setEnabled(ok);
	m_durationSpin->setEnabled(ok);
	m_addBtn->setEnabled(ok);
	rebuildCommandListWidget();
	updateRunStopButtons();
}

void SimulationCommandWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	if (m_hintLabel)
	{
		m_hintLabel->setText(chinese ? QStringLiteral(
								 "为已导入的机器人(URDF)添加关节旋转指令。")
							   : QStringLiteral("Add joint rotation segments for the imported robot (URDF)."));
	}
	m_addBtn->setText(chinese ? QStringLiteral("添加") : QStringLiteral("Add"));
	m_removeBtn->setText(chinese ? QStringLiteral("删除") : QStringLiteral("Remove"));
	m_clearBtn->setText(chinese ? QStringLiteral("清空") : QStringLiteral("Clear"));
	m_runBtn->setText(chinese ? QStringLiteral("运行") : QStringLiteral("Run"));
	m_stopBtn->setText(chinese ? QStringLiteral("停止") : QStringLiteral("Stop"));
	rebuildCommandListWidget();
}

void SimulationCommandWidget::setSimulationRunning(bool running)
{
	m_simulationRunning = running;
	updateRunStopButtons();
}

void SimulationCommandWidget::rebuildCommandListWidget()
{
	m_list->clear();
	for (const RobotSimulationCommand& c : m_commands)
	{
		m_list->addItem(cmdLabel(c, m_jointNames, m_useChinese));
	}
}

void SimulationCommandWidget::onAddClicked()
{
	if (m_jointNames.isEmpty())
	{
		return;
	}
	RobotSimulationCommand c;
	c.jointIndex = m_jointCombo->currentIndex();
	if (c.jointIndex < 0 || c.jointIndex >= m_jointNames.size())
	{
		c.jointIndex = 0;
	}
	c.angleDeg = m_angleSpin->value();
	c.durationSec = m_durationSpin->value();
	m_commands.push_back(c);
	m_list->addItem(cmdLabel(c, m_jointNames, m_useChinese));
	updateRunStopButtons();
}

void SimulationCommandWidget::onRemoveClicked()
{
	const int row = m_list->currentRow();
	if (row >= 0 && row < m_commands.size())
	{
		m_commands.removeAt(row);
		delete m_list->takeItem(row);
		updateRunStopButtons();
	}
}

void SimulationCommandWidget::onClearClicked()
{
	m_commands.clear();
	m_list->clear();
	updateRunStopButtons();
}

QVector<RobotSimulationCommand> SimulationCommandWidget::commands() const
{
	return m_commands;
}