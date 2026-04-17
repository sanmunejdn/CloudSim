#include "SimulationCommandWidget.h"

#include <QComboBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

#include <algorithm>
#include <string>

namespace
{
QString instructionTypeLabel(RobotInstruction::Type t, bool zh)
{
	if (t == RobotInstruction::Type::LINE)
	{
		return zh ? QStringLiteral("直线") : QStringLiteral("LINE");
	}
	return zh ? QStringLiteral("点到点") : QStringLiteral("PTP");
}

double parseDurationSecFromExtension(const RobotInstruction::Base& ins, double fallbackSec)
{
	const auto& ext = ins.extensionProperties();
	const auto it = ext.find("motion.durationSec");
	if (it == ext.end())
	{
		return fallbackSec;
	}
	bool ok = false;
	const double v = QString::fromStdString(it->second).toDouble(&ok);
	return (ok && v > 1e-6) ? v : fallbackSec;
}

QString instructionSummary(const RobotInstruction::Base& ins, bool zh)
{
	const RobotInstruction::Vec3 p = ins.pose();
	const double durationSec = parseDurationSecFromExtension(ins, 0.0);
	const QString durationText = durationSec > 1e-6
		? QString::number(durationSec, 'f', 2)
		: (zh ? QStringLiteral("求解") : QStringLiteral("Solved"));
	return zh
		? QStringLiteral("目标(XYZ): %1, %2, %3 / 时长 %4")
			.arg(p.x, 0, 'f', 1)
			.arg(p.y, 0, 'f', 1)
			.arg(p.z, 0, 'f', 1)
			.arg(durationSec > 1e-6 ? (durationText + QStringLiteral(" s")) : durationText)
		: QStringLiteral("Target(XYZ): %1, %2, %3 / %4")
			.arg(p.x, 0, 'f', 1)
			.arg(p.y, 0, 'f', 1)
			.arg(p.z, 0, 'f', 1)
			.arg(durationSec > 1e-6 ? (durationText + QStringLiteral(" s")) : durationText);
}
} // namespace

SimulationCommandWidget::SimulationCommandWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(6);

	auto* hint = new QLabel(QStringLiteral("Add PTP/LINE instruction from current robot TCP pose."));
	hint->setWordWrap(true);
	root->addWidget(hint);
	m_hintLabel = hint;

	auto* form = new QHBoxLayout;
	m_typeCombo = new QComboBox(this);
	m_typeCombo->addItem(QStringLiteral("PTP"));
	m_typeCombo->addItem(QStringLiteral("LINE"));
	form->addWidget(m_typeCombo);
	root->addLayout(form);

	auto* rowBtns = new QHBoxLayout;
	m_addBtn = new QPushButton(QStringLiteral("Add Instruction"));
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
	connect(m_list, &QListWidget::currentRowChanged, this, [this](int row) {
		if (row < 0 || row >= static_cast<int>(m_instructions.size()))
		{
			emit instructionSelectionChanged(nullptr);
			return;
		}
		emit instructionSelectionChanged(m_instructions[static_cast<size_t>(row)]);
	});
	connect(m_runBtn, &QPushButton::clicked, this, &SimulationCommandWidget::runRequested);
	connect(m_stopBtn, &QPushButton::clicked, this, &SimulationCommandWidget::stopRequested);

	updateRunStopButtons();
}

void SimulationCommandWidget::updateRunStopButtons()
{
	const bool canRun = m_hasRobotContext && !m_instructions.empty();
	m_runBtn->setEnabled(!m_simulationRunning && canRun);
	m_stopBtn->setEnabled(m_simulationRunning);
}

void SimulationCommandWidget::setRevoluteJointNames(const QStringList& names)
{
	m_hasRobotContext = !names.isEmpty();
	m_typeCombo->setEnabled(m_hasRobotContext);
	m_addBtn->setEnabled(m_hasRobotContext);
	rebuildCommandListWidget();
	updateRunStopButtons();
}

void SimulationCommandWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	if (m_hintLabel)
	{
		m_hintLabel->setText(chinese ? QStringLiteral(
								 "点击添加 PTP/LINE，自动读取当前机器人末端位姿。")
							   : QStringLiteral("Add PTP/LINE from current robot TCP pose."));
	}
	m_addBtn->setText(chinese ? QStringLiteral("添加指令") : QStringLiteral("Add Instruction"));
	m_removeBtn->setText(chinese ? QStringLiteral("删除") : QStringLiteral("Remove"));
	m_clearBtn->setText(chinese ? QStringLiteral("清空") : QStringLiteral("Clear"));
	m_runBtn->setText(chinese ? QStringLiteral("运行") : QStringLiteral("Run"));
	m_stopBtn->setText(chinese ? QStringLiteral("停止") : QStringLiteral("Stop"));
	if (m_typeCombo)
	{
		m_typeCombo->setItemText(0, chinese ? QStringLiteral("点到点") : QStringLiteral("PTP"));
		m_typeCombo->setItemText(1, chinese ? QStringLiteral("直线") : QStringLiteral("LINE"));
	}
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
	for (size_t i = 0; i < m_instructions.size(); ++i)
	{
		const auto& ins = m_instructions[i];
		if (!ins)
		{
			continue;
		}
		m_list->addItem(QStringLiteral("[%1] %2")
			.arg(instructionTypeLabel(ins->type(), m_useChinese), instructionSummary(*ins, m_useChinese)));
	}
}

void SimulationCommandWidget::onAddClicked()
{
	if (!m_hasRobotContext)
	{
		return;
	}
	const RobotInstruction::Type t = (m_typeCombo->currentIndex() == 1)
		? RobotInstruction::Type::LINE
		: RobotInstruction::Type::PTP;
	emit addInstructionRequested(t);
}

void SimulationCommandWidget::onRemoveClicked()
{
	const int row = m_list->currentRow();
	if (row >= 0 && row < static_cast<int>(m_instructions.size()))
	{
		m_instructions.erase(m_instructions.begin() + row);
		delete m_list->takeItem(row);
		emit instructionSelectionChanged(nullptr);
		updateRunStopButtons();
	}
}

void SimulationCommandWidget::onClearClicked()
{
	m_instructions.clear();
	m_list->clear();
	emit instructionSelectionChanged(nullptr);
	updateRunStopButtons();
}

QVector<RobotSimulationCommand> SimulationCommandWidget::commands() const
{
	QVector<RobotSimulationCommand> out;
	out.reserve(static_cast<int>(m_instructions.size()));
	for (const auto& ins : m_instructions)
	{
		if (!ins)
		{
			continue;
		}
		RobotSimulationCommand c{};
		c.jointIndex = 0;
		c.angleDeg = 0.0;
		c.durationSec = instructionDurationSec(*ins);
		out.push_back(c);
	}
	return out;
}

std::vector<std::shared_ptr<RobotInstruction::Base>> SimulationCommandWidget::instructions(const QString& controllerId) const
{
	std::vector<std::shared_ptr<RobotInstruction::Base>> out;
	out.reserve(m_instructions.size());
	for (const auto& ins : m_instructions)
	{
		if (!ins)
		{
			continue;
		}
		ins->setControllerId(controllerId.toStdString());
		out.push_back(ins);
	}
	return out;
}

void SimulationCommandWidget::appendInstructionFromCurrentPose(
	RobotInstruction::Type type,
	const RobotInstruction::Vec3& poseMm,
	const RobotInstruction::Vec3& eulerDeg)
{
	std::shared_ptr<RobotInstruction::Base> ins;
	if (type == RobotInstruction::Type::LINE)
	{
		auto line = std::make_shared<RobotInstruction::LineInstruction>();
		line->setBlendRadius(0.0);
		ins = line;
	}
	else
	{
		auto ptp = std::make_shared<RobotInstruction::PtpInstruction>();
		ptp->setAxisConfig("AUTO");
		ins = ptp;
	}
	if (!ins)
	{
		return;
	}
	ins->setPose(poseMm);
	ins->setEulerDeg(eulerDeg);
	m_instructions.push_back(ins);
	rebuildCommandListWidget();
	m_list->setCurrentRow(static_cast<int>(m_instructions.size()) - 1);
	updateRunStopButtons();
}

void SimulationCommandWidget::refreshInstructionList()
{
	rebuildCommandListWidget();
}

void SimulationCommandWidget::clearInstructionSelection()
{
	if (m_list)
	{
		m_list->setCurrentRow(-1);
	}
	emit instructionSelectionChanged(nullptr);
}

double SimulationCommandWidget::instructionDurationSec(const RobotInstruction::Base& ins) const
{
	return parseDurationSecFromExtension(ins, 0.0);
}