/// @file ProcessFlowPropertyPanel.cpp
/// @brief 节点属性面板（按类型显隐）

#include "ProcessFlowPropertyPanel.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>
#include <QLineEdit>
#include <QVBoxLayout>

#include <algorithm>

namespace
{
QDoubleSpinBox* makeSpin(QWidget* parent, double maxV, int decimals = 2)
{
	auto* s = new QDoubleSpinBox(parent);
	s->setRange(0.0, maxV);
	s->setDecimals(decimals);
	return s;
}
} // namespace

ProcessFlowPropertyPanel::ProcessFlowPropertyPanel(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 0, 0, 0);
	root->setSpacing(8);

	m_titleLabel = new QLabel(QStringLiteral("节点属性"), this);
	m_titleLabel->setObjectName(QStringLiteral("ProcessFlowSectionTitle"));

	m_form = new QFormLayout;
	m_form->setContentsMargins(0, 0, 0, 0);
	m_form->setHorizontalSpacing(10);
	m_form->setVerticalSpacing(8);
	m_form->setLabelAlignment(Qt::AlignLeft | Qt::AlignVCenter);
	m_form->setFieldGrowthPolicy(QFormLayout::ExpandingFieldsGrow);

	m_kindCombo = new QComboBox(this);
	for (const QString& kind : ProcessFlowNodeProps::allKinds())
	{
		m_kindCombo->addItem(ProcessFlowNodeProps::displayNameZh(kind), kind);
	}

	m_cycleSpin = makeSpin(this, 1.0e6);
	m_cycleSpin->setSuffix(QStringLiteral(" s"));
	m_inventorySpin = makeSpin(this, 1.0e9);
	m_capacitySpin = makeSpin(this, 1.0e9);
	m_setupSpin = makeSpin(this, 1.0e6);
	m_setupSpin->setSuffix(QStringLiteral(" s"));
	m_prioritySpin = makeSpin(this, 1.0e6, 0);
	m_batchSpin = makeSpin(this, 1.0e6, 0);
	m_batchSpin->setMinimum(1.0);
	m_scrapSpin = makeSpin(this, 1.0, 3);
	m_mtbfSpin = makeSpin(this, 1.0e9);
	m_mtbfSpin->setSuffix(QStringLiteral(" s"));
	m_mttrSpin = makeSpin(this, 1.0e9);
	m_mttrSpin->setSuffix(QStringLiteral(" s"));
	m_inputsSpin = makeSpin(this, 100.0, 0);
	m_inputsSpin->setMinimum(1.0);
	m_bindBackendEdit = new QLineEdit(this);
	m_bindBackendEdit->setPlaceholderText(QStringLiteral("backendId（仅存储）"));
	m_bindProgramEdit = new QLineEdit(this);
	m_bindProgramEdit->setPlaceholderText(QStringLiteral("programId（仅存储）"));

	m_kindLabel = new QLabel(QStringLiteral("类型"), this);
	m_kindLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_timeLabel = new QLabel(QStringLiteral("时间(产能)"), this);
	m_timeLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_invLabel = new QLabel(QStringLiteral("数量(库存)"), this);
	m_invLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_capLabel = new QLabel(QStringLiteral("容量"), this);
	m_capLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_setupLabel = new QLabel(QStringLiteral("换型时间"), this);
	m_setupLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_priorityLabel = new QLabel(QStringLiteral("优先级"), this);
	m_priorityLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_batchLabel = new QLabel(QStringLiteral("批量"), this);
	m_batchLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_scrapLabel = new QLabel(QStringLiteral("报废率"), this);
	m_scrapLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_mtbfLabel = new QLabel(QStringLiteral("MTBF"), this);
	m_mtbfLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_mttrLabel = new QLabel(QStringLiteral("MTTR"), this);
	m_mttrLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_inputsLabel = new QLabel(QStringLiteral("汇合数"), this);
	m_inputsLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_bindBackendLabel = new QLabel(QStringLiteral("绑定Backend"), this);
	m_bindBackendLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));
	m_bindProgramLabel = new QLabel(QStringLiteral("绑定程序"), this);
	m_bindProgramLabel->setObjectName(QStringLiteral("ProcessFlowFieldLabel"));

	m_form->addRow(m_kindLabel, m_kindCombo);
	m_form->addRow(m_timeLabel, m_cycleSpin);
	m_form->addRow(m_invLabel, m_inventorySpin);
	m_form->addRow(m_capLabel, m_capacitySpin);
	m_form->addRow(m_setupLabel, m_setupSpin);
	m_form->addRow(m_priorityLabel, m_prioritySpin);
	m_form->addRow(m_batchLabel, m_batchSpin);
	m_form->addRow(m_scrapLabel, m_scrapSpin);
	m_form->addRow(m_mtbfLabel, m_mtbfSpin);
	m_form->addRow(m_mttrLabel, m_mttrSpin);
	m_form->addRow(m_inputsLabel, m_inputsSpin);
	m_form->addRow(m_bindBackendLabel, m_bindBackendEdit);
	m_form->addRow(m_bindProgramLabel, m_bindProgramEdit);

	root->addWidget(m_titleLabel);
	root->addLayout(m_form);
	root->addStretch(1);

	connect(m_kindCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this,
			&ProcessFlowPropertyPanel::onKindChanged);
	auto bindSpin = [this](QDoubleSpinBox* s)
	{ connect(s, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, [this](double) { emitEdited(); }); };
	bindSpin(m_cycleSpin);
	bindSpin(m_inventorySpin);
	bindSpin(m_capacitySpin);
	bindSpin(m_setupSpin);
	bindSpin(m_prioritySpin);
	bindSpin(m_batchSpin);
	bindSpin(m_scrapSpin);
	bindSpin(m_mtbfSpin);
	bindSpin(m_mttrSpin);
	bindSpin(m_inputsSpin);
	connect(m_bindBackendEdit, &QLineEdit::textChanged, this, [this](const QString&) { emitEdited(); });
	connect(m_bindProgramEdit, &QLineEdit::textChanged, this, [this](const QString&) { emitEdited(); });

	clearSelection();
}

void ProcessFlowPropertyPanel::setSpin(QDoubleSpinBox* spin, double v)
{
	spin->setValue(v);
}

void ProcessFlowPropertyPanel::updateFieldVisibility(const QString& kind)
{
	const bool isStart = kind == QStringLiteral("start");
	const bool isEnd = kind == QStringLiteral("end");
	const bool isBuf = ProcessFlowNodeProps::isBufferKind(kind);
	const bool isMachine = ProcessFlowNodeProps::isMachineKind(kind);
	const bool isInspect = kind == QStringLiteral("inspect");
	const bool isAssembly = kind == QStringLiteral("assembly");
	const bool isConveyor = kind == QStringLiteral("conveyor") || kind == QStringLiteral("agv");

	auto setRow = [](QLabel* lab, QWidget* w, bool on)
	{
		lab->setVisible(on);
		w->setVisible(on);
	};

	if (isEnd)
	{
		setRow(m_timeLabel, m_cycleSpin, false);
		setRow(m_invLabel, m_inventorySpin, false);
		setRow(m_capLabel, m_capacitySpin, false);
		setRow(m_setupLabel, m_setupSpin, false);
		setRow(m_priorityLabel, m_prioritySpin, true);
		setRow(m_batchLabel, m_batchSpin, false);
		setRow(m_scrapLabel, m_scrapSpin, false);
		setRow(m_mtbfLabel, m_mtbfSpin, false);
		setRow(m_mttrLabel, m_mttrSpin, false);
		setRow(m_inputsLabel, m_inputsSpin, false);
		setRow(m_bindBackendLabel, m_bindBackendEdit, false);
		setRow(m_bindProgramLabel, m_bindProgramEdit, false);
		return;
	}

	setRow(m_timeLabel, m_cycleSpin, isStart || isMachine);
	setRow(m_invLabel, m_inventorySpin, isBuf);
	setRow(m_capLabel, m_capacitySpin, isBuf || isMachine);
	setRow(m_setupLabel, m_setupSpin, kind == QStringLiteral("station") || isInspect || isAssembly);
	setRow(m_priorityLabel, m_prioritySpin, true);
	setRow(m_batchLabel, m_batchSpin, kind == QStringLiteral("station") || isAssembly);
	setRow(m_scrapLabel, m_scrapSpin, isInspect);
	setRow(m_mtbfLabel, m_mtbfSpin, isMachine);
	setRow(m_mttrLabel, m_mttrSpin, isMachine);
	setRow(m_inputsLabel, m_inputsSpin, isAssembly);
	setRow(m_bindBackendLabel, m_bindBackendEdit, isMachine);
	setRow(m_bindProgramLabel, m_bindProgramEdit, isMachine);

	if (isStart)
	{
		m_timeLabel->setText(m_useChinese ? QStringLiteral("到达间隔") : QStringLiteral("Interarrival"));
	}
	else if (isConveyor)
	{
		m_timeLabel->setText(m_useChinese ? QStringLiteral("运输时间") : QStringLiteral("Transport Time"));
	}
	else
	{
		m_timeLabel->setText(m_useChinese ? QStringLiteral("时间(产能)") : QStringLiteral("Cycle Time"));
	}
}

void ProcessFlowPropertyPanel::applyLanguage(bool useChinese)
{
	m_useChinese = useChinese;
	m_titleLabel->setText(useChinese ? QStringLiteral("节点属性") : QStringLiteral("Node Properties"));
	m_kindLabel->setText(useChinese ? QStringLiteral("类型") : QStringLiteral("Type"));
	m_invLabel->setText(useChinese ? QStringLiteral("数量(库存)") : QStringLiteral("Inventory"));
	m_capLabel->setText(useChinese ? QStringLiteral("容量") : QStringLiteral("Capacity"));
	m_setupLabel->setText(useChinese ? QStringLiteral("换型时间") : QStringLiteral("Setup Time"));
	m_priorityLabel->setText(useChinese ? QStringLiteral("优先级") : QStringLiteral("Priority"));
	m_batchLabel->setText(useChinese ? QStringLiteral("批量") : QStringLiteral("Batch Size"));
	m_scrapLabel->setText(useChinese ? QStringLiteral("报废率") : QStringLiteral("Scrap Rate"));
	m_mtbfLabel->setText(QStringLiteral("MTBF"));
	m_mttrLabel->setText(QStringLiteral("MTTR"));
	m_inputsLabel->setText(useChinese ? QStringLiteral("汇合数") : QStringLiteral("Required Inputs"));

	const QString curKind = m_kindCombo->currentData().toString();
	m_block = true;
	m_kindCombo->clear();
	for (const QString& kind : ProcessFlowNodeProps::allKinds())
	{
		m_kindCombo->addItem(useChinese ? ProcessFlowNodeProps::displayNameZh(kind)
										: ProcessFlowNodeProps::displayNameEn(kind),
							 kind);
	}
	const int idx = m_kindCombo->findData(curKind);
	if (idx >= 0)
	{
		m_kindCombo->setCurrentIndex(idx);
	}
	m_block = false;
	updateFieldVisibility(m_kindCombo->currentData().toString());
}

void ProcessFlowPropertyPanel::clearSelection()
{
	m_nodeId = -1;
	m_block = true;
	setEnabled(false);
	m_kindCombo->setCurrentIndex(0);
	setSpin(m_cycleSpin, 0.0);
	setSpin(m_inventorySpin, 0.0);
	setSpin(m_capacitySpin, 0.0);
	setSpin(m_setupSpin, 0.0);
	setSpin(m_prioritySpin, 0.0);
	setSpin(m_batchSpin, 1.0);
	setSpin(m_scrapSpin, 0.0);
	setSpin(m_mtbfSpin, 0.0);
	setSpin(m_mttrSpin, 0.0);
	setSpin(m_inputsSpin, 2.0);
	m_bindBackendEdit->clear();
	m_bindProgramEdit->clear();
	m_block = false;
	updateFieldVisibility(m_kindCombo->currentData().toString());
}

void ProcessFlowPropertyPanel::setNodeProps(int nodeId, const ProcessFlowNodeProps& props)
{
	m_nodeId = nodeId;
	setEnabled(nodeId >= 0);
	if (nodeId < 0)
	{
		return;
	}
	m_block = true;
	const int idx = m_kindCombo->findData(props.kind);
	m_kindCombo->setCurrentIndex(idx >= 0 ? idx : 0);
	setSpin(m_cycleSpin, props.cycleTimeSec);
	setSpin(m_inventorySpin, props.inventoryQty);
	setSpin(m_capacitySpin, props.capacityQty);
	setSpin(m_setupSpin, props.setupTimeSec);
	setSpin(m_prioritySpin, props.priority);
	setSpin(m_batchSpin, std::max(1.0, props.batchSize));
	setSpin(m_scrapSpin, props.scrapRate);
	setSpin(m_mtbfSpin, props.mtbfSec);
	setSpin(m_mttrSpin, props.mttrSec);
	setSpin(m_inputsSpin, std::max(1.0, props.requiredInputs));
	m_bindBackendEdit->setText(props.bindingBackendId);
	m_bindProgramEdit->setText(props.bindingProgramId);
	m_block = false;
	updateFieldVisibility(props.kind);
}

void ProcessFlowPropertyPanel::onKindChanged(int)
{
	if (m_block || m_nodeId < 0)
	{
		return;
	}
	const QString kind = m_kindCombo->currentData().toString();
	const ProcessFlowNodeProps defaults = ProcessFlowNodeProps::defaultsForKind(kind);
	m_block = true;
	setSpin(m_cycleSpin, defaults.cycleTimeSec);
	setSpin(m_inventorySpin, defaults.inventoryQty);
	setSpin(m_capacitySpin, defaults.capacityQty);
	setSpin(m_setupSpin, defaults.setupTimeSec);
	setSpin(m_prioritySpin, defaults.priority);
	setSpin(m_batchSpin, defaults.batchSize);
	setSpin(m_scrapSpin, defaults.scrapRate);
	setSpin(m_mtbfSpin, defaults.mtbfSec);
	setSpin(m_mttrSpin, defaults.mttrSec);
	setSpin(m_inputsSpin, defaults.requiredInputs);
	m_block = false;
	updateFieldVisibility(kind);
	emitEdited();
}

void ProcessFlowPropertyPanel::emitEdited()
{
	if (m_block || m_nodeId < 0)
	{
		return;
	}
	ProcessFlowNodeProps props;
	props.kind = m_kindCombo->currentData().toString();
	props.cycleTimeSec = m_cycleSpin->value();
	props.inventoryQty = m_inventorySpin->value();
	props.capacityQty = m_capacitySpin->value();
	props.setupTimeSec = m_setupSpin->value();
	props.priority = m_prioritySpin->value();
	props.batchSize = m_batchSpin->value();
	props.scrapRate = m_scrapSpin->value();
	props.mtbfSec = m_mtbfSpin->value();
	props.mttrSec = m_mttrSpin->value();
	props.requiredInputs = m_inputsSpin->value();
	props.bindingBackendId = m_bindBackendEdit->text().trimmed();
	props.bindingProgramId = m_bindProgramEdit->text().trimmed();
	emit propsEdited(m_nodeId, props);
}
