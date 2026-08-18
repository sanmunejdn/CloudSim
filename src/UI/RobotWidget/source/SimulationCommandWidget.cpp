/// @file SimulationCommandWidget.cpp
/// @brief 仿真指令坞

#include "SimulationCommandWidget.h"

#include "InstructionProgramTreeWidget.h"
#include "ProgramEditCommand.h"
#include "ProgramEditService.h"
#include "RobotInstructionFactory.h"
#include "RobotInstructionProgram.h"
#include "RobotProgramStore.h"
#include "UiIconDecorators.h"

#include <QComboBox>
#include <QFrame>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QInputDialog>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QSignalBlocker>
#include <QSizePolicy>
#include <QString>
#include <QStyle>
#include <QToolTip>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <string>

namespace
{
constexpr int kControlHeight = 30;
constexpr int kContextLabelWidth = 40;
constexpr int kTypeBtnMinWidth = 52;

void applyBtnRole(QPushButton* btn, const char* role)
{
	if (!btn)
	{
		return;
	}
	btn->setProperty("btnRole", QLatin1String(role));
	if (btn->style())
	{
		btn->style()->unpolish(btn);
		btn->style()->polish(btn);
	}
}

void configureCompactCombo(QComboBox* combo)
{
	if (!combo)
	{
		return;
	}
	combo->setFixedHeight(kControlHeight);
	combo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	combo->setMaxVisibleItems(12);
}

void configureCompactButton(QPushButton* btn, int minWidth = 0)
{
	if (!btn)
	{
		return;
	}
	// 全局 padding 4px + 1px 边框，高度过小会裁掉下边框
	btn->setFixedHeight(kControlHeight);
	btn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
	if (minWidth > 0)
	{
		btn->setMinimumWidth(minWidth);
	}
}

QFrame* makeHLine(QWidget* parent)
{
	auto* line = new QFrame(parent);
	line->setFrameShape(QFrame::HLine);
	line->setFrameShadow(QFrame::Sunken);
	line->setFixedHeight(1);
	return line;
}

QString instructionTypeLabel(RobotInstruction::Type t, bool zh)
{
	switch (t)
	{
	case RobotInstruction::Type::LINE:
		return zh ? QStringLiteral("直线") : QStringLiteral("LINE");
	case RobotInstruction::Type::ARC:
		return zh ? QStringLiteral("圆弧") : QStringLiteral("ARC");
	case RobotInstruction::Type::WAIT:
		return zh ? QStringLiteral("等待") : QStringLiteral("WAIT");
	case RobotInstruction::Type::IF:
		return zh ? QStringLiteral("条件") : QStringLiteral("IF");
	case RobotInstruction::Type::WHILE:
		return zh ? QStringLiteral("循环") : QStringLiteral("WHILE");
	case RobotInstruction::Type::SET_DO:
		return zh ? QStringLiteral("DO") : QStringLiteral("SET_DO");
	case RobotInstruction::Type::SET_AO:
		return zh ? QStringLiteral("AO") : QStringLiteral("SET_AO");
	case RobotInstruction::Type::PathPlan:
		return zh ? QStringLiteral("路径") : QStringLiteral("PATH");
	case RobotInstruction::Type::DeviceAxis:
		return zh ? QStringLiteral("设备轴") : QStringLiteral("DEV_AXIS");
	case RobotInstruction::Type::PTP:
	default:
		return zh ? QStringLiteral("PTP") : QStringLiteral("PTP");
	}
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

QString conditionSummary(const RobotInstruction::Condition& c, bool zh)
{
	using RobotInstruction::ConditionKind;
	switch (c.kind)
	{
	case ConditionKind::Never:
		return zh ? QStringLiteral("永不") : QStringLiteral("never");
	case ConditionKind::Io:
		if (!c.signalName.empty())
		{
			return zh ? QStringLiteral("%1==%2").arg(QString::fromStdString(c.signalName)).arg(c.ioEquals ? 1 : 0)
					  : QStringLiteral("%1==%2").arg(QString::fromStdString(c.signalName)).arg(c.ioEquals ? 1 : 0);
		}
		return zh ? QStringLiteral("IO%1==%2").arg(c.ioPort).arg(c.ioEquals ? 1 : 0)
				  : QStringLiteral("IO%1==%2").arg(c.ioPort).arg(c.ioEquals ? 1 : 0);
	case ConditionKind::Compare:
		return QString::fromStdString(c.compareLeft + " " + c.compareOp + " " + std::to_string(c.compareRight));
	case ConditionKind::Always:
	default:
		return zh ? QStringLiteral("始终") : QStringLiteral("always");
	}
}

QString instructionSummary(const RobotInstruction::Base& ins, bool zh)
{
	switch (ins.type())
	{
	case RobotInstruction::Type::WAIT:
	{
		const RobotInstruction::Condition& c = ins.condition();
		if (c.kind == RobotInstruction::ConditionKind::Io)
		{
			const QString sig = !c.signalName.empty() ? QString::fromStdString(c.signalName)
													 : QStringLiteral("IO%1").arg(c.ioPort);
			const QString cond = QStringLiteral("%1==%2").arg(sig).arg(c.ioEquals ? 1 : 0);
			if (ins.durationSec() > 1e-9)
			{
				return zh ? QStringLiteral("等 %1（超时 %2 s）").arg(cond).arg(ins.durationSec(), 0, 'f', 2)
						  : QStringLiteral("wait %1 (timeout %2 s)").arg(cond).arg(ins.durationSec(), 0, 'f', 2);
			}
			return zh ? QStringLiteral("等 %1").arg(cond) : QStringLiteral("wait %1").arg(cond);
		}
		return zh ? QStringLiteral("时长 %1 s").arg(ins.durationSec(), 0, 'f', 2)
				  : QStringLiteral("duration %1 s").arg(ins.durationSec(), 0, 'f', 2);
	}
	case RobotInstruction::Type::IF:
	case RobotInstruction::Type::WHILE:
		return conditionSummary(ins.condition(), zh);
	case RobotInstruction::Type::SET_DO:
		if (!ins.ioSignalName().empty())
		{
			return zh ? QStringLiteral("%1 = %2")
							.arg(QString::fromStdString(ins.ioSignalName()))
							.arg(ins.ioBoolValue() ? 1 : 0)
					  : QStringLiteral("%1 = %2")
							.arg(QString::fromStdString(ins.ioSignalName()))
							.arg(ins.ioBoolValue() ? 1 : 0);
		}
		return zh ? QStringLiteral("端口 %1 = %2").arg(ins.ioPort()).arg(ins.ioBoolValue() ? 1 : 0)
				  : QStringLiteral("port %1 = %2").arg(ins.ioPort()).arg(ins.ioBoolValue() ? 1 : 0);
	case RobotInstruction::Type::SET_AO:
		if (!ins.ioSignalName().empty())
		{
			return zh ? QStringLiteral("%1 = %2")
							.arg(QString::fromStdString(ins.ioSignalName()))
							.arg(ins.ioAnalogValue(), 0, 'f', 2)
					  : QStringLiteral("%1 = %2")
							.arg(QString::fromStdString(ins.ioSignalName()))
							.arg(ins.ioAnalogValue(), 0, 'f', 2);
		}
		return zh ? QStringLiteral("端口 %1 = %2").arg(ins.ioPort()).arg(ins.ioAnalogValue(), 0, 'f', 2)
				  : QStringLiteral("port %1 = %2").arg(ins.ioPort()).arg(ins.ioAnalogValue(), 0, 'f', 2);
	case RobotInstruction::Type::DeviceAxis:
		return zh ? QStringLiteral("%1 轴%2 → %3")
						.arg(QString::fromStdString(ins.deviceBackendId()))
						.arg(ins.deviceAxisIndex())
						.arg(ins.deviceAxisTargetQ(), 0, 'g', 6)
				  : QStringLiteral("%1 axis%2 -> %3")
						.arg(QString::fromStdString(ins.deviceBackendId()))
						.arg(ins.deviceAxisIndex())
						.arg(ins.deviceAxisTargetQ(), 0, 'g', 6);
	case RobotInstruction::Type::PTP:
	case RobotInstruction::Type::LINE:
	case RobotInstruction::Type::ARC:
	{
		const double durationSec = parseDurationSecFromExtension(ins, 0.0);
		const QString durationText = durationSec > 1e-6 ? QString::number(durationSec, 'f', 2)
														: (zh ? QStringLiteral("求解") : QStringLiteral("Solved"));
		const QString motionText = QString::fromStdString(RobotInstruction::formatMotionWaypointSummary(ins, zh));
		return zh ? QStringLiteral("%1 / 时长 %2")
						.arg(motionText)
						.arg(durationSec > 1e-6 ? (durationText + QStringLiteral(" s")) : durationText)
				  : QStringLiteral("%1 / %2")
						.arg(motionText)
						.arg(durationSec > 1e-6 ? (durationText + QStringLiteral(" s")) : durationText);
	}
	default:
		break;
	}
	return QString();
}
} // namespace

SimulationCommandWidget::SimulationCommandWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(4);

	auto* rowRun = new QHBoxLayout;
	rowRun->setSpacing(4);
	m_runBtn = new QPushButton(QStringLiteral("Run"));
	m_stopBtn = new QPushButton(QStringLiteral("Stop"));
	m_exportBtn = new QPushButton(QStringLiteral("Export"));
	m_stopBtn->setEnabled(false);
	m_playbackRateLabel = new QLabel(QStringLiteral("Sim Rate"), this);
	m_playbackRateCombo = new QComboBox(this);
	const double ratePresets[] = {0.1, 0.25, 0.5, 1.0, 2.0, 5.0, 10.0};
	int defaultRateIndex = 3;
	for (int i = 0; i < static_cast<int>(sizeof(ratePresets) / sizeof(ratePresets[0])); ++i)
	{
		const double r = ratePresets[i];
		const QString label = (r < 1.0) ? QString::number(r, 'g', 3) + QStringLiteral("×")
										: QString::number(static_cast<int>(r)) + QStringLiteral("×");
		m_playbackRateCombo->addItem(label, r);
		if (std::abs(r - 1.0) < 1e-9)
		{
			defaultRateIndex = i;
		}
	}
	m_playbackRateCombo->setCurrentIndex(defaultRateIndex);
	m_playbackRateCombo->setToolTip(QStringLiteral("Playback rate (does not change planned duration)"));
	configureCompactButton(m_runBtn);
	configureCompactButton(m_stopBtn);
	configureCompactButton(m_exportBtn);
	configureCompactCombo(m_playbackRateCombo);
	m_playbackRateCombo->setMinimumWidth(64);
	m_playbackRateCombo->setMaximumWidth(88);
	applyBtnRole(m_runBtn, "primary");
	applyBtnRole(m_stopBtn, "danger");
	applyBtnRole(m_exportBtn, "secondary");
	rowRun->addWidget(m_runBtn);
	rowRun->addWidget(m_stopBtn);
	rowRun->addWidget(m_playbackRateLabel);
	rowRun->addWidget(m_playbackRateCombo);
	rowRun->addStretch(1);
	rowRun->addWidget(m_exportBtn);
	root->addLayout(rowRun);
	root->addWidget(makeHLine(this));

	m_robotLabel = new QLabel(QStringLiteral("机器人"), this);
	m_robotLabel->setFixedWidth(kContextLabelWidth);
	m_robotCombo = new QComboBox(this);
	configureCompactCombo(m_robotCombo);
	auto* robotRow = new QHBoxLayout;
	robotRow->setSpacing(4);
	robotRow->addWidget(m_robotLabel);
	robotRow->addWidget(m_robotCombo, 1);
	root->addLayout(robotRow);

	// 界面隐藏，仍由 setTcpLinkOptions 维护 preferred
	m_tcpLinkCombo = new QComboBox(this);
	m_tcpLinkCombo->hide();

	auto* programRow = new QHBoxLayout;
	programRow->setSpacing(2);
	m_programLabel = new QLabel(QStringLiteral("程序"), this);
	m_programLabel->setFixedWidth(kContextLabelWidth);
	programRow->addWidget(m_programLabel);
	m_programCombo = new QComboBox(this);
	configureCompactCombo(m_programCombo);
	programRow->addWidget(m_programCombo, 1);
	m_programNewBtn = new QPushButton(QStringLiteral("新建"), this);
	m_programRenameBtn = new QPushButton(QStringLiteral("重命名"), this);
	m_programDeleteBtn = new QPushButton(QStringLiteral("删除"), this);
	configureCompactButton(m_programNewBtn);
	configureCompactButton(m_programRenameBtn);
	configureCompactButton(m_programDeleteBtn);
	applyBtnRole(m_programNewBtn, "secondary");
	applyBtnRole(m_programRenameBtn, "secondary");
	applyBtnRole(m_programDeleteBtn, "danger");
	programRow->addWidget(m_programNewBtn);
	programRow->addWidget(m_programRenameBtn);
	programRow->addWidget(m_programDeleteBtn);
	root->addLayout(programRow);

	const RobotInstruction::Type motionTypes[] = {
		RobotInstruction::Type::PTP,
		RobotInstruction::Type::LINE,
		RobotInstruction::Type::ARC,
	};
	const RobotInstruction::Type logicTypes[] = {
		RobotInstruction::Type::WAIT,		 RobotInstruction::Type::IF,		 RobotInstruction::Type::WHILE,
		RobotInstruction::Type::SET_DO,		 RobotInstruction::Type::SET_AO,	 RobotInstruction::Type::DeviceAxis,
		RobotInstruction::Type::PathPlan,
	};

	auto* insertBar = new QWidget(this);
	auto* insertOuter = new QHBoxLayout(insertBar);
	insertOuter->setContentsMargins(0, 0, 0, 0);
	insertOuter->setSpacing(4);
	m_insertLabel = new QLabel(QStringLiteral("插入"), insertBar);
	m_insertLabel->setFixedWidth(kContextLabelWidth);
	insertOuter->addWidget(m_insertLabel, 0, Qt::AlignTop);

	auto* insertBtnsHost = new QWidget(insertBar);
	auto* insertGrid = new QGridLayout(insertBtnsHost);
	insertGrid->setContentsMargins(0, 1, 0, 1);
	insertGrid->setHorizontalSpacing(2);
	insertGrid->setVerticalSpacing(4);
	int col = 0;
	for (const RobotInstruction::Type t : motionTypes)
	{
		auto* typeBtn = createTypeButton(t);
		configureCompactButton(typeBtn, kTypeBtnMinWidth);
		applyBtnRole(typeBtn, "secondary");
		insertGrid->addWidget(typeBtn, 0, col++);
	}
	col = 0;
	for (const RobotInstruction::Type t : logicTypes)
	{
		auto* typeBtn = createTypeButton(t);
		configureCompactButton(typeBtn, kTypeBtnMinWidth);
		applyBtnRole(typeBtn, "secondary");
		insertGrid->addWidget(typeBtn, 1, col++);
	}
	insertGrid->setColumnStretch(col, 1);
	insertOuter->addWidget(insertBtnsHost, 1);
	root->addWidget(insertBar);

	auto* editBar = new QWidget(this);
	auto* editCol = new QVBoxLayout(editBar);
	editCol->setContentsMargins(0, 1, 0, 1);
	editCol->setSpacing(2);
	auto* funcRow = new QHBoxLayout;
	funcRow->setSpacing(2);
	m_editLabel = new QLabel(QStringLiteral("编辑"), editBar);
	m_editLabel->setFixedWidth(kContextLabelWidth);
	funcRow->addWidget(m_editLabel);
	m_tcpDragTeachBtn = new QPushButton(QStringLiteral("Drag"), editBar);
	m_tcpDragTeachBtn->setCheckable(true);
	configureCompactButton(m_tcpDragTeachBtn, kTypeBtnMinWidth);
	applyBtnRole(m_tcpDragTeachBtn, "secondary");
	connect(m_tcpDragTeachBtn, &QPushButton::toggled, this,
			[this](const bool checked)
			{
				if (m_tcpDragTeachMode == checked)
				{
					updateTcpDragTeachUi(checked);
					return;
				}
				if (checked && m_waypointPickMode)
				{
					setInstructionWaypointPickMode(false);
					emit instructionWaypointPickModeChanged(false);
				}
				m_tcpDragTeachMode = checked;
				updateTcpDragTeachUi(checked);
				emit tcpDragTeachModeChanged(checked);
			});
	funcRow->addWidget(m_tcpDragTeachBtn);
	m_waypointPickBtn = new QPushButton(QStringLiteral("Pick"), editBar);
	m_waypointPickBtn->setCheckable(true);
	configureCompactButton(m_waypointPickBtn, kTypeBtnMinWidth);
	applyBtnRole(m_waypointPickBtn, "secondary");
	connect(m_waypointPickBtn, &QPushButton::toggled, this,
			[this](const bool checked)
			{
				if (m_waypointPickMode == checked)
				{
					updateWaypointPickUi(checked);
					return;
				}
				if (checked && m_tcpDragTeachMode)
				{
					setTcpDragTeachMode(false);
					emit tcpDragTeachModeChanged(false);
				}
				m_waypointPickMode = checked;
				updateWaypointPickUi(checked);
				emit instructionWaypointPickModeChanged(checked);
			});
	funcRow->addWidget(m_waypointPickBtn);
	m_removeBtn = new QPushButton(QStringLiteral("Remove"), editBar);
	m_clearBtn = new QPushButton(QStringLiteral("Clear"), editBar);
	configureCompactButton(m_removeBtn, kTypeBtnMinWidth);
	configureCompactButton(m_clearBtn, kTypeBtnMinWidth);
	applyBtnRole(m_removeBtn, "danger");
	applyBtnRole(m_clearBtn, "danger");
	funcRow->addWidget(m_removeBtn);
	funcRow->addWidget(m_clearBtn);
	funcRow->addStretch(1);
	editCol->addLayout(funcRow);
	m_tcpDragHintLabel = new QLabel(editBar);
	m_tcpDragHintLabel->setWordWrap(true);
	m_tcpDragHintLabel->setVisible(false);
	m_tcpDragHintLabel->setStyleSheet(QStringLiteral("color: #288cf0;"));
	editCol->addWidget(m_tcpDragHintLabel);
	root->addWidget(editBar);
	root->addWidget(makeHLine(this));

	m_tree = new InstructionProgramTreeWidget(this);
	root->addWidget(m_tree, 1);

	connect(m_robotCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&SimulationCommandWidget::onRobotComboChanged);
	connect(m_programCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&SimulationCommandWidget::onProgramComboChanged);
	connect(m_programNewBtn, &QPushButton::clicked, this, &SimulationCommandWidget::onProgramNewClicked);
	connect(m_programRenameBtn, &QPushButton::clicked, this, &SimulationCommandWidget::onProgramRenameClicked);
	connect(m_programDeleteBtn, &QPushButton::clicked, this, &SimulationCommandWidget::onProgramDeleteClicked);
	connect(m_removeBtn, &QPushButton::clicked, this, &SimulationCommandWidget::onRemoveClicked);
	connect(m_clearBtn, &QPushButton::clicked, this, &SimulationCommandWidget::onClearClicked);
	connect(m_tree, &InstructionProgramTreeWidget::instructionSelected, this,
			&SimulationCommandWidget::instructionSelectionChanged);
	connect(m_tree, &InstructionProgramTreeWidget::programStructureChanged, this, [this]() { updateRunStopButtons(); });
	connect(m_tree, &InstructionProgramTreeWidget::groupMembershipChanged, this, [this]() { emit groupsChanged(); });
	connect(m_tree, &InstructionProgramTreeWidget::createGroupRequested, this,
			&SimulationCommandWidget::onCreateGroupRequested);
	connect(m_tree, &InstructionProgramTreeWidget::dissolveGroupRequested, this,
			&SimulationCommandWidget::onDissolveGroupRequested);
	connect(m_tree, &InstructionProgramTreeWidget::renameGroupRequested, this,
			&SimulationCommandWidget::onRenameGroupRequested);
	connect(m_runBtn, &QPushButton::clicked, this, &SimulationCommandWidget::runRequested);
	connect(m_stopBtn, &QPushButton::clicked, this, &SimulationCommandWidget::stopRequested);
	connect(m_exportBtn, &QPushButton::clicked, this, &SimulationCommandWidget::exportProgramRequested);
	connect(m_playbackRateCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			[this](int)
			{
				emit playbackRateChanged(playbackRate());
			});

	UiIconDecorators::apply(m_tcpDragTeachBtn, UiIconId::TcpDragTeach);
	UiIconDecorators::apply(m_waypointPickBtn, UiIconId::PickFace);
	UiIconDecorators::apply(m_removeBtn, UiIconId::Delete);
	UiIconDecorators::apply(m_clearBtn, UiIconId::Clear);
	UiIconDecorators::apply(m_runBtn, UiIconId::Run, UiIconDecorators::IconPlacement::Leading, UiIcons::Size::Medium);
	UiIconDecorators::apply(m_stopBtn, UiIconId::Stop, UiIconDecorators::IconPlacement::Leading, UiIcons::Size::Medium);
	UiIconDecorators::apply(m_exportBtn, UiIconId::Export);

	updateTypeButtonLabels();
	updateRunStopButtons();
	setUseChinese(m_useChinese);
}

QPushButton* SimulationCommandWidget::createTypeButton(const RobotInstruction::Type type)
{
	auto* btn = new QPushButton(this);
	btn->setProperty("instructionType", static_cast<int>(type));
	btn->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
	connect(btn, &QPushButton::clicked, this, &SimulationCommandWidget::onAddTypeButtonClicked);
	m_typeButtons.push_back(btn);
	return btn;
}

void SimulationCommandWidget::setProgramStore(RobotProgramStore* store)
{
	if (m_tree)
	{
		m_tree->setProgram(nullptr);
	}
	m_programStore = store;
	refreshProgramCombo();
}

void SimulationCommandWidget::setProgramEditService(ProgramEditService* service)
{
	m_editService = service;
}

void SimulationCommandWidget::bindProgramTree()
{
	if (!m_tree)
	{
		return;
	}
	// 仅同步语言标志；重建由 rebuildCommandListWidget 统一完成，避免 setProgram 前对悬空 m_program renumber
	m_tree->setUseChinese(m_useChinese);
	if (!m_programStore)
	{
		m_tree->setProgram(nullptr);
		return;
	}
	const QString backendId = m_programStore->activeRobotBackendId();
	if (backendId.isEmpty())
	{
		m_tree->setProgram(nullptr);
		return;
	}
	m_tree->setProgram(&m_programStore->programFor(backendId));
}

void SimulationCommandWidget::setRobotInstances(const QStringList& labels, const QStringList& backendIds)
{
	if (m_programStore)
	{
		m_programStore->setRobotInstances(labels, backendIds);
	}
	m_robotCombo->blockSignals(true);
	m_robotCombo->clear();
	for (const QString& label : labels)
	{
		m_robotCombo->addItem(label);
	}
	if (m_robotCombo->count() > 0)
	{
		m_robotCombo->setCurrentIndex(0);
	}
	m_robotCombo->blockSignals(false);
	if (m_robotCombo->count() > 0)
	{
		onRobotComboChanged(0);
	}
	else
	{
		// 无实例时不会走 onRobotComboChanged，须主动清空指令树（切到空文档/新建）
		refreshProgramCombo();
		refreshInstructionList();
	}
}

int SimulationCommandWidget::currentRobotInstanceIndex() const
{
	return m_robotCombo ? m_robotCombo->currentIndex() : -1;
}

QString SimulationCommandWidget::currentRobotBackendId() const
{
	if (m_programStore)
	{
		return m_programStore->activeRobotBackendId();
	}
	return QString();
}

void SimulationCommandWidget::onRobotComboChanged(const int index)
{
	setArcTeachPending(false);
	if (!m_programStore || index < 0)
	{
		return;
	}
	m_programStore->setActiveInstanceIndex(index);
	refreshProgramCombo();
	refreshInstructionList();
	emit robotSelectionChanged(index, m_programStore->activeRobotBackendId());
}

void SimulationCommandWidget::updateRunStopButtons()
{
	const bool hasProg = m_programStore && !m_programStore->activeProgram().empty();
	const bool canRun = m_hasRobotContext && hasProg;
	m_runBtn->setEnabled(!m_simulationRunning && canRun);
	m_stopBtn->setEnabled(m_simulationRunning);
}

void SimulationCommandWidget::setRevoluteJointNames(const QStringList& names)
{
	m_hasRobotContext = !names.isEmpty();
	setTypeButtonsEnabled(m_hasRobotContext);
	if (m_tcpLinkCombo)
	{
		m_tcpLinkCombo->setEnabled(m_hasRobotContext);
	}
	m_robotCombo->setEnabled(m_hasRobotContext);
	updateRunStopButtons();
}

void SimulationCommandWidget::setTypeButtonsEnabled(const bool enabled)
{
	for (QPushButton* btn : m_typeButtons)
	{
		if (btn)
		{
			btn->setEnabled(enabled);
		}
	}
	if (m_tcpDragTeachBtn)
	{
		m_tcpDragTeachBtn->setEnabled(enabled && !m_simulationRunning);
	}
	if (m_waypointPickBtn)
	{
		m_waypointPickBtn->setEnabled(enabled && !m_simulationRunning);
	}
}

void SimulationCommandWidget::setTcpLinkOptions(const QStringList& linkNames, const QString& preferredLink)
{
	if (!m_tcpLinkCombo)
	{
		return;
	}
	const QString prev = selectedTcpLink();
	m_tcpLinkCombo->clear();
	for (const QString& n : linkNames)
	{
		m_tcpLinkCombo->addItem(n);
	}
	if (m_tcpLinkCombo->count() <= 0)
	{
		return;
	}
	int idx = -1;
	if (!preferredLink.isEmpty())
	{
		idx = m_tcpLinkCombo->findText(preferredLink);
	}
	if (idx < 0 && !prev.isEmpty())
	{
		idx = m_tcpLinkCombo->findText(prev);
	}
	if (idx < 0)
	{
		idx = 0;
	}
	m_tcpLinkCombo->setCurrentIndex(idx);
}

QString SimulationCommandWidget::selectedTcpLink() const
{
	if (!m_tcpLinkCombo || m_tcpLinkCombo->currentIndex() < 0)
	{
		return QString();
	}
	return m_tcpLinkCombo->currentText();
}

double SimulationCommandWidget::playbackRate() const
{
	if (!m_playbackRateCombo)
	{
		return 1.0;
	}
	const QVariant data = m_playbackRateCombo->currentData();
	bool ok = false;
	const double rate = data.toDouble(&ok);
	return (ok && rate > 0.0) ? rate : 1.0;
}

void SimulationCommandWidget::setUseChinese(bool chinese)
{
	m_useChinese = chinese;
	updateProgramGroupUi();
	if (m_insertLabel)
	{
		m_insertLabel->setText(chinese ? QStringLiteral("插入") : QStringLiteral("Insert"));
	}
	if (m_editLabel)
	{
		m_editLabel->setText(chinese ? QStringLiteral("编辑") : QStringLiteral("Edit"));
	}
	m_removeBtn->setText(chinese ? QStringLiteral("删除") : QStringLiteral("Remove"));
	m_clearBtn->setText(chinese ? QStringLiteral("清空") : QStringLiteral("Clear"));
	m_runBtn->setText(chinese ? QStringLiteral("运行") : QStringLiteral("Run"));
	m_stopBtn->setText(chinese ? QStringLiteral("停止") : QStringLiteral("Stop"));
	if (m_playbackRateLabel)
	{
		m_playbackRateLabel->setText(chinese ? QStringLiteral("仿真倍率") : QStringLiteral("Sim Rate"));
	}
	if (m_playbackRateCombo)
	{
		m_playbackRateCombo->setToolTip(chinese ? QStringLiteral("播放倍率（不改变规划时长）")
												: QStringLiteral("Playback rate (does not change planned duration)"));
	}
	if (m_exportBtn)
	{
		m_exportBtn->setText(chinese ? QStringLiteral("导出") : QStringLiteral("Export"));
	}
	if (m_tcpDragTeachBtn)
	{
		m_tcpDragTeachBtn->setText(chinese ? QStringLiteral("拖动") : QStringLiteral("Drag"));
		m_tcpDragTeachBtn->setToolTip(
			chinese ? QStringLiteral("在 3D 视图中拖动 TCP 罗盘示教（不写指令）")
					: QStringLiteral("Drag TCP gizmo in 3D view to teach pose (does not edit program)"));
	}
	if (m_waypointPickBtn)
	{
		m_waypointPickBtn->setText(chinese ? QStringLiteral("拾取") : QStringLiteral("Pick"));
		m_waypointPickBtn->setToolTip(chinese ? QStringLiteral("在 3D 中点击路点，指令树跳转到该指令")
											 : QStringLiteral("Click a waypoint in 3D to select it in the tree"));
	}
	updateTcpDragTeachUi(m_tcpDragTeachMode);
	updateWaypointPickUi(m_waypointPickMode);
	updateTypeButtonLabels();
	rebuildCommandListWidget();
}

void SimulationCommandWidget::updateProgramGroupUi()
{
	const bool zh = m_useChinese;
	const int labelW = zh ? kContextLabelWidth : 56;
	if (m_robotLabel)
	{
		m_robotLabel->setText(zh ? QStringLiteral("机器人") : QStringLiteral("Robot"));
		m_robotLabel->setFixedWidth(labelW);
	}
	if (m_programLabel)
	{
		m_programLabel->setText(zh ? QStringLiteral("程序") : QStringLiteral("Program"));
		m_programLabel->setFixedWidth(labelW);
	}
	if (m_insertLabel)
	{
		m_insertLabel->setFixedWidth(labelW);
	}
	if (m_editLabel)
	{
		m_editLabel->setFixedWidth(labelW);
	}
	if (m_programNewBtn)
	{
		m_programNewBtn->setText(zh ? QStringLiteral("新建") : QStringLiteral("New"));
		m_programNewBtn->setToolTip(zh ? QStringLiteral("新建程序") : QStringLiteral("New program"));
	}
	if (m_programRenameBtn)
	{
		m_programRenameBtn->setText(zh ? QStringLiteral("重命名") : QStringLiteral("Rename"));
		m_programRenameBtn->setToolTip(zh ? QStringLiteral("重命名") : QStringLiteral("Rename"));
	}
	if (m_programDeleteBtn)
	{
		m_programDeleteBtn->setText(zh ? QStringLiteral("删除") : QStringLiteral("Delete"));
		m_programDeleteBtn->setToolTip(zh ? QStringLiteral("删除程序") : QStringLiteral("Delete program"));
	}
}

void SimulationCommandWidget::updateTypeButtonLabels()
{
	const bool zh = m_useChinese;
	for (QPushButton* btn : m_typeButtons)
	{
		if (!btn)
		{
			continue;
		}
		const auto t = static_cast<RobotInstruction::Type>(btn->property("instructionType").toInt());
		btn->setText(instructionTypeLabel(t, zh));
		if (t == RobotInstruction::Type::PTP)
		{
			UiIconDecorators::apply(btn, UiIconId::Ptp, UiIconDecorators::IconPlacement::Leading, UiIcons::Size::Small);
		}
		else if (t == RobotInstruction::Type::LINE)
		{
			UiIconDecorators::apply(btn, UiIconId::Line, UiIconDecorators::IconPlacement::Leading,
									UiIcons::Size::Small);
		}
		else if (t == RobotInstruction::Type::ARC)
		{
			UiIconDecorators::apply(btn, UiIconId::Line, UiIconDecorators::IconPlacement::Leading,
									UiIcons::Size::Small);
		}
		QString tip;
		switch (t)
		{
		case RobotInstruction::Type::LINE:
			tip = zh ? QStringLiteral("插入直线运动（使用当前 TCP 位姿）")
					 : QStringLiteral("Insert LINE motion from current TCP pose");
			break;
		case RobotInstruction::Type::ARC:
			tip = m_arcTeachPending
					  ? (zh ? QStringLiteral("再点一次：确认圆弧终点（当前 TCP）")
							: QStringLiteral("Click again: confirm ARC end (current TCP)"))
					  : (zh ? QStringLiteral("圆弧两步示教：先捕获途经点 Via，再确认终点 End")
							: QStringLiteral("ARC two-step teach: capture Via, then End"));
			break;
		case RobotInstruction::Type::WAIT:
			tip = zh ? QStringLiteral("插入等待：延时，或等到 DI 信号（可设超时）")
					 : QStringLiteral("Insert WAIT: delay or wait for DI signal (optional timeout)");
			break;
		case RobotInstruction::Type::IF:
			tip = zh ? QStringLiteral("插入条件分支（可按 DI 信号）")
					 : QStringLiteral("Insert IF with Then/Else (DI signal supported)");
			break;
		case RobotInstruction::Type::WHILE:
			tip = zh ? QStringLiteral("插入循环（可按 DI 信号）") : QStringLiteral("Insert WHILE (DI signal supported)");
			break;
		case RobotInstruction::Type::SET_DO:
			tip = zh ? QStringLiteral("插入数字量输出") : QStringLiteral("Insert digital output");
			break;
		case RobotInstruction::Type::SET_AO:
			tip = zh ? QStringLiteral("插入模拟量输出") : QStringLiteral("Insert analog output");
			break;
		case RobotInstruction::Type::DeviceAxis:
			tip = zh ? QStringLiteral("插入自定义设备轴运动") : QStringLiteral("Insert custom device axis motion");
			break;
		case RobotInstruction::Type::PathPlan:
			tip = zh ? QStringLiteral("插入路径规划（流水线与离散 raw）")
					 : QStringLiteral("Insert path plan (pipeline + discrete raw)");
			break;
		case RobotInstruction::Type::PTP:
		default:
			tip = zh ? QStringLiteral("插入点到点运动（使用当前 TCP 位姿）")
					 : QStringLiteral("Insert PTP motion from current TCP pose");
			break;
		}
		btn->setToolTip(tip);
	}
	refreshArcTeachButtonLabels();
}

void SimulationCommandWidget::refreshArcTeachButtonLabels()
{
	const bool zh = m_useChinese;
	for (QPushButton* btn : m_typeButtons)
	{
		if (!btn)
		{
			continue;
		}
		const auto t = static_cast<RobotInstruction::Type>(btn->property("instructionType").toInt());
		if (t != RobotInstruction::Type::ARC)
		{
			continue;
		}
		if (m_arcTeachPending)
		{
			btn->setText(zh ? QStringLiteral("确认终点") : QStringLiteral("Confirm End"));
			btn->setToolTip(zh ? QStringLiteral("再点一次：确认圆弧终点（当前 TCP）")
							   : QStringLiteral("Click again: confirm ARC end (current TCP)"));
		}
		else
		{
			btn->setText(instructionTypeLabel(t, zh));
		}
	}
}

void SimulationCommandWidget::setArcTeachPending(bool pending)
{
	if (m_arcTeachPending == pending)
	{
		return;
	}
	m_arcTeachPending = pending;
	refreshArcTeachButtonLabels();
}

void SimulationCommandWidget::setSimulationRunning(bool running)
{
	m_simulationRunning = running;
	if (running && m_tcpDragTeachBtn && m_tcpDragTeachBtn->isChecked())
	{
		const QSignalBlocker b(m_tcpDragTeachBtn);
		m_tcpDragTeachBtn->setChecked(false);
		m_tcpDragTeachMode = false;
		updateTcpDragTeachUi(false);
		emit tcpDragTeachModeChanged(false);
	}
	if (running && m_waypointPickBtn && m_waypointPickBtn->isChecked())
	{
		const QSignalBlocker b(m_waypointPickBtn);
		m_waypointPickBtn->setChecked(false);
		m_waypointPickMode = false;
		updateWaypointPickUi(false);
		emit instructionWaypointPickModeChanged(false);
	}
	if (m_tcpDragTeachBtn)
	{
		m_tcpDragTeachBtn->setEnabled(!running && m_hasRobotContext);
	}
	if (m_waypointPickBtn)
	{
		m_waypointPickBtn->setEnabled(!running && m_hasRobotContext);
	}
	updateRunStopButtons();
}

void SimulationCommandWidget::setTcpDragTeachMode(const bool enabled)
{
	if (!m_tcpDragTeachBtn)
	{
		m_tcpDragTeachMode = enabled;
		return;
	}
	if (m_tcpDragTeachBtn->isChecked() == enabled)
	{
		m_tcpDragTeachMode = enabled;
		updateTcpDragTeachUi(enabled);
		return;
	}
	const QSignalBlocker b(m_tcpDragTeachBtn);
	m_tcpDragTeachBtn->setChecked(enabled);
	m_tcpDragTeachMode = enabled;
	updateTcpDragTeachUi(enabled);
}

void SimulationCommandWidget::updateTcpDragTeachUi(const bool enabled)
{
	if (m_tcpDragTeachBtn)
	{
		applyBtnRole(m_tcpDragTeachBtn, enabled ? "primary" : "secondary");
	}
	refreshEditModeHint();
}

void SimulationCommandWidget::setInstructionWaypointPickMode(const bool enabled)
{
	if (!m_waypointPickBtn)
	{
		m_waypointPickMode = enabled;
		return;
	}
	if (m_waypointPickBtn->isChecked() == enabled)
	{
		m_waypointPickMode = enabled;
		updateWaypointPickUi(enabled);
		return;
	}
	const QSignalBlocker b(m_waypointPickBtn);
	m_waypointPickBtn->setChecked(enabled);
	m_waypointPickMode = enabled;
	updateWaypointPickUi(enabled);
}

void SimulationCommandWidget::updateWaypointPickUi(const bool enabled)
{
	if (m_waypointPickBtn)
	{
		applyBtnRole(m_waypointPickBtn, enabled ? "primary" : "secondary");
	}
	refreshEditModeHint();
}

void SimulationCommandWidget::refreshEditModeHint()
{
	if (!m_tcpDragHintLabel)
	{
		return;
	}
	if (m_tcpDragTeachMode)
	{
		m_tcpDragHintLabel->setText(
			m_useChinese ? QStringLiteral("拖动示教中：在 3D 视图拖动 TCP，再次点击「拖动」退出")
						 : QStringLiteral("TCP teach on: drag gizmo in 3D view; click Drag again to exit"));
		m_tcpDragHintLabel->setVisible(true);
		if (m_tcpDragTeachBtn)
		{
			QToolTip::showText(m_tcpDragTeachBtn->mapToGlobal(QPoint(0, m_tcpDragTeachBtn->height())),
							   m_tcpDragHintLabel->text(), m_tcpDragTeachBtn);
		}
		return;
	}
	if (m_waypointPickMode)
	{
		m_tcpDragHintLabel->setText(m_useChinese ? QStringLiteral("拾取路点：在 3D 中点击路径点，Esc 退出")
												 : QStringLiteral("Pick waypoint: click a path point in 3D; Esc to exit"));
		m_tcpDragHintLabel->setVisible(true);
		if (m_waypointPickBtn)
		{
			QToolTip::showText(m_waypointPickBtn->mapToGlobal(QPoint(0, m_waypointPickBtn->height())),
							   m_tcpDragHintLabel->text(), m_waypointPickBtn);
		}
		return;
	}
	m_tcpDragHintLabel->clear();
	m_tcpDragHintLabel->setVisible(false);
	QToolTip::hideText();
}

bool SimulationCommandWidget::tcpDragTeachMode() const
{
	return m_tcpDragTeachMode;
}

bool SimulationCommandWidget::instructionWaypointPickMode() const
{
	return m_waypointPickMode;
}

void SimulationCommandWidget::rebuildCommandListWidget()
{
	bindProgramTree();
	if (m_tree && m_programStore)
	{
		const std::string activeId = m_programStore->activeProgramIdUtf8();
		RobotInstruction::RobotProgram* prog = m_programStore->activeCatalog().findProgram(activeId);
		if (prog)
		{
			m_tree->setGroupMembership(&prog->groups);
		}
		else
		{
			m_tree->setGroupMembership(nullptr);
		}
		m_tree->rebuildFromProgram();
	}
}

void SimulationCommandWidget::refreshProgramCombo()
{
	if (!m_programCombo || !m_programStore)
	{
		return;
	}
	const std::string activeId = m_programStore->activeProgramIdUtf8();
	const auto& catalog = m_programStore->activeCatalog();
	m_programCombo->blockSignals(true);
	m_programCombo->clear();
	int activeIdx = 0;
	for (size_t i = 0; i < catalog.programs().size(); ++i)
	{
		const RobotInstruction::RobotProgram& prog = catalog.programs()[i];
		m_programCombo->addItem(QString::fromStdString(prog.name), QString::fromStdString(prog.id));
		if (prog.id == activeId)
		{
			activeIdx = static_cast<int>(i);
		}
	}
	m_programCombo->setCurrentIndex(activeIdx);
	m_programCombo->blockSignals(false);
}

void SimulationCommandWidget::onProgramComboChanged(int index)
{
	if (!m_programStore || index < 0)
	{
		return;
	}
	const std::string programId = m_programCombo->itemData(index).toString().toStdString();
	m_programStore->setActiveProgramIdUtf8(programId);
	refreshInstructionList();
	emit activeProgramChanged(QString::fromStdString(programId));
}

void SimulationCommandWidget::onProgramNewClicked()
{
	if (!m_programStore)
	{
		return;
	}
	bool ok = false;
	const bool zh = m_useChinese;
	const QString name = QInputDialog::getText(this, zh ? QStringLiteral("新建程序") : QStringLiteral("New program"),
											   zh ? QStringLiteral("名称") : QStringLiteral("Name"), QLineEdit::Normal,
											   zh ? QStringLiteral("程序") : QStringLiteral("Program"), &ok);
	if (!ok || name.isEmpty())
	{
		return;
	}
	RobotInstruction::RobotProgram prog;
	const std::string newId = RobotInstruction::makeProgramId();
	prog.id = newId;
	prog.name = name.toStdString();
	std::string err;
	if (!m_programStore->activeCatalog().addProgram(std::move(prog), &err))
	{
		return;
	}
	refreshProgramCombo();
	const int idx = m_programCombo->findData(QString::fromStdString(newId));
	if (idx >= 0)
	{
		m_programCombo->setCurrentIndex(idx);
	}
}

void SimulationCommandWidget::onProgramRenameClicked()
{
	if (!m_programStore || !m_programCombo)
	{
		return;
	}
	const std::string programId = m_programCombo->currentData().toString().toStdString();
	bool ok = false;
	const bool zh = m_useChinese;
	const QString name = QInputDialog::getText(
		this, zh ? QStringLiteral("重命名程序") : QStringLiteral("Rename program"),
		zh ? QStringLiteral("名称") : QStringLiteral("Name"), QLineEdit::Normal, m_programCombo->currentText(), &ok);
	if (!ok || name.isEmpty())
	{
		return;
	}
	std::string err;
	m_programStore->activeCatalog().renameProgram(programId, name.toStdString(), &err);
	refreshProgramCombo();
}

void SimulationCommandWidget::onProgramDeleteClicked()
{
	if (!m_programStore || !m_programCombo)
	{
		return;
	}
	const std::string programId = m_programCombo->currentData().toString().toStdString();
	std::string err;
	if (!m_programStore->activeCatalog().removeProgram(programId, &err))
	{
		return;
	}
	refreshProgramCombo();
	refreshInstructionList();
	emit activeProgramChanged(QString::fromStdString(m_programStore->activeProgramIdUtf8()));
}

void SimulationCommandWidget::onCreateGroupRequested(const QString& name, const std::vector<std::string>& memberIds)
{
	if (!m_programStore || !m_editService || memberIds.empty())
	{
		return;
	}
	RobotInstruction::RobotProgram* prog =
		m_programStore->activeCatalog().findProgram(m_programStore->activeProgramIdUtf8());
	if (!prog)
	{
		return;
	}
	auto cmd = std::make_unique<RobotInstruction::CreateInstructionGroupCommand>(prog, name.toStdString(), memberIds);
	QString err;
	if (!m_editService->execute(std::shared_ptr<RobotInstruction::ProgramEditCommand>(std::move(cmd)), &err))
	{
		return;
	}
	rebuildCommandListWidget();
	emit groupsChanged();
}

void SimulationCommandWidget::onDissolveGroupRequested(const std::string& groupId)
{
	if (!m_programStore || !m_editService || groupId.empty())
	{
		return;
	}
	RobotInstruction::RobotProgram* prog =
		m_programStore->activeCatalog().findProgram(m_programStore->activeProgramIdUtf8());
	if (!prog)
	{
		return;
	}
	auto cmd = std::make_unique<RobotInstruction::RemoveInstructionGroupCommand>(prog, groupId);
	QString err;
	if (!m_editService->execute(std::shared_ptr<RobotInstruction::ProgramEditCommand>(std::move(cmd)), &err))
	{
		return;
	}
	rebuildCommandListWidget();
	emit groupsChanged();
}

void SimulationCommandWidget::onRenameGroupRequested(const std::string& groupId, const QString& newName)
{
	if (!m_programStore || !m_editService || groupId.empty() || newName.isEmpty())
	{
		return;
	}
	RobotInstruction::RobotProgram* prog =
		m_programStore->activeCatalog().findProgram(m_programStore->activeProgramIdUtf8());
	if (!prog)
	{
		return;
	}
	auto cmd = std::make_unique<RobotInstruction::RenameInstructionGroupCommand>(prog, groupId, newName.toStdString());
	QString err;
	if (!m_editService->execute(std::shared_ptr<RobotInstruction::ProgramEditCommand>(std::move(cmd)), &err))
	{
		return;
	}
	rebuildCommandListWidget();
	emit groupsChanged();
}

void SimulationCommandWidget::onAddTypeButtonClicked()
{
	auto* btn = qobject_cast<QPushButton*>(sender());
	if (!btn)
	{
		return;
	}
	const auto t = static_cast<RobotInstruction::Type>(btn->property("instructionType").toInt());
	requestAddInstruction(t);
}

void SimulationCommandWidget::requestAddInstruction(const RobotInstruction::Type type)
{
	if (!m_hasRobotContext || !m_programStore)
	{
		return;
	}
	if (type == RobotInstruction::Type::PTP || type == RobotInstruction::Type::LINE ||
		type == RobotInstruction::Type::ARC)
	{
		emit addInstructionRequested(type);
		return;
	}
	(void)appendInstruction(type);
}

void SimulationCommandWidget::onRemoveClicked()
{
	if (!m_tree || !m_programStore)
	{
		return;
	}
	const std::shared_ptr<RobotInstruction::Base> sel = m_tree->selectedInstruction();
	if (sel && sel->type() == RobotInstruction::Type::PathPlan && m_editService)
	{
		const std::string programId = m_programStore->activeProgramIdUtf8();
		std::shared_ptr<RobotInstruction::ProgramEditCommand> cmd =
			std::make_shared<RobotInstruction::RemovePathPlanCommand>(&m_programStore->activeCatalog(), programId,
																	  sel->id());
		QString err;
		if (m_editService->execute(cmd, &err))
		{
			rebuildCommandListWidget();
			emit instructionSelectionChanged(nullptr);
		}
		updateRunStopButtons();
		return;
	}
	m_tree->removeSelected();
	updateRunStopButtons();
}

void SimulationCommandWidget::onClearClicked()
{
	setArcTeachPending(false);
	if (m_tree)
	{
		m_tree->clearProgram();
		updateRunStopButtons();
	}
}

QVector<RobotSimulationCommand> SimulationCommandWidget::commands() const
{
	QVector<RobotSimulationCommand> out;
	if (!m_programStore)
	{
		return out;
	}
	const auto& instructions = m_programStore->activeProgram();
	out.reserve(static_cast<int>(instructions.size()));
	for (const auto& ins : instructions)
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

std::vector<std::shared_ptr<RobotInstruction::Base>>
SimulationCommandWidget::instructions(const QString& robotBackendId) const
{
	if (!m_programStore)
	{
		return {};
	}
	std::vector<std::shared_ptr<RobotInstruction::Base>> out = m_programStore->programFor(robotBackendId);
	for (const auto& ins : out)
	{
		if (ins)
		{
			ins->setControllerId(robotBackendId.toStdString());
		}
	}
	return out;
}

std::vector<std::shared_ptr<RobotInstruction::Base>> SimulationCommandWidget::instructionList() const
{
	if (!m_programStore)
	{
		return {};
	}
	std::vector<std::shared_ptr<RobotInstruction::Base>> flat;
	RobotInstruction::flattenInstructionsRecursive(m_programStore->activeProgram(), flat);
	return flat;
}

std::vector<std::string> SimulationCommandWidget::selectedMotionInstructionIds() const
{
	std::vector<std::string> ids;
	if (!m_tree)
	{
		return ids;
	}
	for (const std::shared_ptr<RobotInstruction::Base>& ins : m_tree->selectedMotionInstructions())
	{
		if (ins)
		{
			ids.push_back(ins->id());
		}
	}
	return ids;
}

std::shared_ptr<RobotInstruction::Base> SimulationCommandWidget::appendInstruction(const RobotInstruction::Type type)
{
	if (!m_programStore)
	{
		return nullptr;
	}
	const QString robotId = m_programStore->activeRobotBackendId();
	std::shared_ptr<RobotInstruction::Base> ins;
	switch (type)
	{
	case RobotInstruction::Type::LINE:
		ins = std::make_shared<RobotInstruction::LineInstruction>();
		break;
	case RobotInstruction::Type::ARC:
		ins = std::make_shared<RobotInstruction::ArcInstruction>();
		break;
	case RobotInstruction::Type::WAIT:
		ins = std::make_shared<RobotInstruction::WaitInstruction>();
		{
			RobotInstruction::Condition c;
			c.kind = RobotInstruction::ConditionKind::Io;
			c.ioEquals = true;
			ins->setCondition(c);
			ins->setDurationSec(0.0);
		}
		break;
	case RobotInstruction::Type::IF:
		ins = std::make_shared<RobotInstruction::IfInstruction>();
		{
			RobotInstruction::Condition c;
			c.kind = RobotInstruction::ConditionKind::Io;
			ins->setCondition(c);
		}
		break;
	case RobotInstruction::Type::WHILE:
		ins = std::make_shared<RobotInstruction::WhileInstruction>();
		{
			RobotInstruction::Condition c;
			c.kind = RobotInstruction::ConditionKind::Io;
			ins->setCondition(c);
		}
		break;
	case RobotInstruction::Type::SET_DO:
		ins = std::make_shared<RobotInstruction::SetDigitalOutputInstruction>();
		break;
	case RobotInstruction::Type::SET_AO:
		ins = std::make_shared<RobotInstruction::SetAnalogOutputInstruction>();
		break;
	case RobotInstruction::Type::DeviceAxis:
		ins = std::make_shared<RobotInstruction::DeviceAxisInstruction>();
		break;
	case RobotInstruction::Type::PathPlan:
	{
		auto pathPlan = std::make_shared<RobotInstruction::PathPlanInstruction>();
		if (m_editService && m_programStore)
		{
			size_t pathPlanInsertIdx = 0;
			for (const std::shared_ptr<RobotInstruction::Base>& step : m_programStore->activeProgram())
			{
				if (step && step->type() == RobotInstruction::Type::PathPlan)
				{
					++pathPlanInsertIdx;
				}
			}
			std::shared_ptr<RobotInstruction::ProgramEditCommand> cmd =
				std::make_shared<RobotInstruction::InsertPathPlanCommand>(pathPlan, pathPlanInsertIdx);
			QString err;
			if (m_editService->execute(cmd, &err))
			{
				rebuildCommandListWidget();
				emit instructionSelectionChanged(pathPlan);
				updateRunStopButtons();
				return pathPlan;
			}
		}
		ins = pathPlan;
		break;
	}
	case RobotInstruction::Type::PTP:
	default:
		ins = std::make_shared<RobotInstruction::PtpInstruction>();
		break;
	}
	if (!ins)
	{
		return nullptr;
	}
	if (!robotId.isEmpty())
	{
		ins->setControllerId(robotId.toStdString());
	}
	if (m_tree)
	{
		m_tree->insertInstruction(ins);
	}
	else
	{
		m_programStore->activeProgram().push_back(ins);
		rebuildCommandListWidget();
	}
	updateRunStopButtons();
	return ins;
}

bool SimulationCommandWidget::appendInstructionFromJson(const nlohmann::json& j, std::string* errMsg)
{
	std::string localErr;
	auto ins = RobotInstruction::createFromJson(j, errMsg ? errMsg : &localErr);
	if (!ins || !m_programStore)
	{
		return false;
	}
	const QString robotId = m_programStore->activeRobotBackendId();
	if (!robotId.isEmpty())
	{
		ins->setControllerId(robotId.toStdString());
	}
	if (m_tree)
	{
		m_tree->insertInstruction(ins);
	}
	else
	{
		m_programStore->activeProgram().push_back(std::move(ins));
		rebuildCommandListWidget();
	}
	updateRunStopButtons();
	return true;
}

std::shared_ptr<RobotInstruction::Base> SimulationCommandWidget::appendInstructionFromCurrentPose(
	RobotInstruction::Type type, const RobotInstruction::Vec3& poseMm, const RobotInstruction::Vec3& eulerDeg,
	const bool deferInstructionSelection)
{
	std::shared_ptr<RobotInstruction::Base> ins;
	if (type == RobotInstruction::Type::LINE)
	{
		ins = std::make_shared<RobotInstruction::LineInstruction>();
	}
	else
	{
		auto ptp = std::make_shared<RobotInstruction::PtpInstruction>();
		ptp->setAxisConfig("AUTO");
		ins = ptp;
	}
	if (!ins || !m_programStore)
	{
		return nullptr;
	}
	ins->setPose(poseMm);
	ins->setEulerDeg(eulerDeg);
	const QString robotId = m_programStore->activeRobotBackendId();
	if (!robotId.isEmpty())
	{
		ins->setControllerId(robotId.toStdString());
	}
	if (m_tree)
	{
		m_tree->insertInstruction(ins, !deferInstructionSelection);
	}
	else
	{
		m_programStore->activeProgram().push_back(ins);
		rebuildCommandListWidget();
	}
	updateRunStopButtons();
	return ins;
}

std::shared_ptr<RobotInstruction::Base> SimulationCommandWidget::appendArcInstructionFromPoses(
	const RobotInstruction::Vec3& viaPoseMm, const RobotInstruction::Vec3& viaEulerDeg,
	const RobotInstruction::Vec3& endPoseMm, const RobotInstruction::Vec3& endEulerDeg,
	const bool deferInstructionSelection)
{
	if (!m_programStore)
	{
		return nullptr;
	}
	auto arc = std::make_shared<RobotInstruction::ArcInstruction>();
	arc->setViaPose(viaPoseMm);
	arc->setViaEulerDeg(viaEulerDeg);
	arc->setPose(endPoseMm);
	arc->setEulerDeg(endEulerDeg);
	const QString robotId = m_programStore->activeRobotBackendId();
	if (!robotId.isEmpty())
	{
		arc->setControllerId(robotId.toStdString());
	}
	std::shared_ptr<RobotInstruction::Base> ins = arc;
	if (m_tree)
	{
		m_tree->insertInstruction(ins, !deferInstructionSelection);
	}
	else
	{
		m_programStore->activeProgram().push_back(ins);
		rebuildCommandListWidget();
	}
	updateRunStopButtons();
	return ins;
}

void SimulationCommandWidget::refreshInstructionList()
{
	rebuildCommandListWidget();
	updateRunStopButtons();
}

void SimulationCommandWidget::clearInstructionSelection()
{
	if (m_tree)
	{
		m_tree->clearSelection();
	}
	emit instructionSelectionChanged(nullptr);
}

double SimulationCommandWidget::instructionDurationSec(const RobotInstruction::Base& ins) const
{
	if (ins.hasDurationProperty())
	{
		return ins.durationSec();
	}
	return parseDurationSecFromExtension(ins, 0.0);
}
