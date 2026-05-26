#include "TrajectoryEditPageWidget.h"

#include "ProgramEditService.h"
#include "SimulationCommandWidget.h"
#include "TrajectoryEditSession.h"
#include "TrajectoryPipelineListWidget.h"
#include "RobotProgramStore.h"

#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QVBoxLayout>

#include <QDrag>
#include <QMimeData>

#include <cstring>

namespace
{
/// 调色板拖放须携带 kMimeType，否则流水线只会出现“幽灵项”（有显示无数据）
class TrajectoryOpPaletteWidget : public QListWidget
{
public:
	explicit TrajectoryOpPaletteWidget(QWidget* parent = nullptr)
		: QListWidget(parent)
	{
		setDragEnabled(true);
	}

protected:
	void startDrag(Qt::DropActions supportedActions) override
	{
		(void)supportedActions;
		QListWidgetItem* item = currentItem();
		if (!item)
		{
			return;
		}
		const int kindInt = item->data(Qt::UserRole).toInt();
		QByteArray raw;
		raw.resize(static_cast<int>(sizeof(int)));
		std::memcpy(raw.data(), &kindInt, sizeof(int));
		auto* mime = new QMimeData();
		mime->setData(TrajectoryPipelineListWidget::kMimeType, raw);
		auto* drag = new QDrag(this);
		drag->setMimeData(mime);
		drag->exec(Qt::CopyAction);
	}
};
} // namespace

namespace
{
QString opKindLabel(RobotInstruction::TrajectoryOpKind kind, bool zh)
{
	switch (kind)
	{
	case RobotInstruction::TrajectoryOpKind::Rotate:
		return zh ? QStringLiteral("旋转") : QStringLiteral("Rotate");
	case RobotInstruction::TrajectoryOpKind::Mirror:
		return zh ? QStringLiteral("镜像") : QStringLiteral("Mirror");
	case RobotInstruction::TrajectoryOpKind::Delete:
		return zh ? QStringLiteral("删除") : QStringLiteral("Delete");
	case RobotInstruction::TrajectoryOpKind::Duplicate:
		return zh ? QStringLiteral("复制") : QStringLiteral("Duplicate");
	case RobotInstruction::TrajectoryOpKind::Reorder:
		return zh ? QStringLiteral("移动顺序") : QStringLiteral("Reorder");
	case RobotInstruction::TrajectoryOpKind::Translate:
	default:
		return zh ? QStringLiteral("平移") : QStringLiteral("Translate");
	}
}
} // namespace

TrajectoryEditPageWidget::TrajectoryEditPageWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(6);

	auto* scopeRow = new QHBoxLayout;
	m_programLabel = new QLabel(QStringLiteral("程序"), this);
	scopeRow->addWidget(m_programLabel);
	m_programCombo = new QComboBox(this);
	scopeRow->addWidget(m_programCombo, 1);
	m_groupLabel = new QLabel(QStringLiteral("分组"), this);
	scopeRow->addWidget(m_groupLabel);
	m_groupCombo = new QComboBox(this);
	scopeRow->addWidget(m_groupCombo, 1);
	root->addLayout(scopeRow);

	auto* bodyRow = new QHBoxLayout;
	m_palette = new TrajectoryOpPaletteWidget(this);
	m_palette->setFixedWidth(120);
	bodyRow->addWidget(m_palette);

	m_pipeline = new TrajectoryPipelineListWidget(this);
	m_pipeline->setDefaultOpFactory([this](const RobotInstruction::TrajectoryOpKind kind) {
		return makeDefaultOp(kind);
	});
	bodyRow->addWidget(m_pipeline, 1);
	root->addLayout(bodyRow, 1);

	m_paramGroupBox = new QGroupBox(QStringLiteral("参数"), this);
	auto* paramForm = new QFormLayout(m_paramGroupBox);
	m_scopeKindCombo = new QComboBox(m_paramGroupBox);
	m_scopeKindFieldLabel = new QLabel(QStringLiteral("作用域"), m_paramGroupBox);
	paramForm->addRow(m_scopeKindFieldLabel, m_scopeKindCombo);
	m_scopeGroupCombo = new QComboBox(m_paramGroupBox);
	m_scopeGroupFieldLabel = new QLabel(QStringLiteral("分组"), m_paramGroupBox);
	paramForm->addRow(m_scopeGroupFieldLabel, m_scopeGroupCombo);
	auto* rangeRow = new QHBoxLayout;
	m_pointFromSpin = new QDoubleSpinBox(m_paramGroupBox);
	m_pointFromSpin->setRange(1, 9999);
	m_pointFromSpin->setDecimals(0);
	m_pointToSpin = new QDoubleSpinBox(m_paramGroupBox);
	m_pointToSpin->setRange(1, 9999);
	m_pointToSpin->setDecimals(0);
	rangeRow->addWidget(m_pointFromSpin);
	rangeRow->addWidget(new QLabel(QStringLiteral("-"), m_paramGroupBox));
	rangeRow->addWidget(m_pointToSpin);
	m_pointRangeFieldLabel = new QLabel(QStringLiteral("P"), m_paramGroupBox);
	paramForm->addRow(m_pointRangeFieldLabel, rangeRow);
	m_dxSpin = new QDoubleSpinBox(m_paramGroupBox);
	m_dySpin = new QDoubleSpinBox(m_paramGroupBox);
	m_dzSpin = new QDoubleSpinBox(m_paramGroupBox);
	for (QDoubleSpinBox* spin : { m_dxSpin, m_dySpin, m_dzSpin })
	{
		spin->setRange(-100000.0, 100000.0);
		spin->setDecimals(2);
	}
	m_dxFieldLabel = new QLabel(QStringLiteral("ΔX mm"), m_paramGroupBox);
	m_dyFieldLabel = new QLabel(QStringLiteral("ΔY mm"), m_paramGroupBox);
	m_dzFieldLabel = new QLabel(QStringLiteral("ΔZ mm"), m_paramGroupBox);
	paramForm->addRow(m_dxFieldLabel, m_dxSpin);
	paramForm->addRow(m_dyFieldLabel, m_dySpin);
	paramForm->addRow(m_dzFieldLabel, m_dzSpin);
	m_axisXSpin = new QDoubleSpinBox(m_paramGroupBox);
	m_axisYSpin = new QDoubleSpinBox(m_paramGroupBox);
	m_axisZSpin = new QDoubleSpinBox(m_paramGroupBox);
	m_angleSpin = new QDoubleSpinBox(m_paramGroupBox);
	for (QDoubleSpinBox* spin : { m_axisXSpin, m_axisYSpin, m_axisZSpin })
	{
		spin->setRange(-1.0, 1.0);
		spin->setDecimals(3);
	}
	m_axisZSpin->setValue(1.0);
	m_angleSpin->setRange(-360.0, 360.0);
	m_angleSpin->setDecimals(2);
	m_axisXFieldLabel = new QLabel(QStringLiteral("轴 X"), m_paramGroupBox);
	m_axisYFieldLabel = new QLabel(QStringLiteral("轴 Y"), m_paramGroupBox);
	m_axisZFieldLabel = new QLabel(QStringLiteral("轴 Z"), m_paramGroupBox);
	m_angleFieldLabel = new QLabel(QStringLiteral("角度 °"), m_paramGroupBox);
	paramForm->addRow(m_axisXFieldLabel, m_axisXSpin);
	paramForm->addRow(m_axisYFieldLabel, m_axisYSpin);
	paramForm->addRow(m_axisZFieldLabel, m_axisZSpin);
	paramForm->addRow(m_angleFieldLabel, m_angleSpin);
	m_mirrorHintLabel = new QLabel(m_paramGroupBox);
	m_mirrorHintLabel->setWordWrap(true);
	m_mirrorHintLabel->setStyleSheet(QStringLiteral("color: gray;"));
	paramForm->addRow(m_mirrorHintLabel);
	root->addWidget(m_paramGroupBox);

	auto* opRow = new QHBoxLayout;
	m_removeOpBtn = new QPushButton(QStringLiteral("移除块"), this);
	m_moveUpBtn = new QPushButton(QStringLiteral("上移"), this);
	m_moveDownBtn = new QPushButton(QStringLiteral("下移"), this);
	opRow->addWidget(m_removeOpBtn);
	opRow->addWidget(m_moveUpBtn);
	opRow->addWidget(m_moveDownBtn);
	opRow->addStretch(1);
	root->addLayout(opRow);

	auto* actionRow = new QHBoxLayout;
	m_previewBtn = new QPushButton(QStringLiteral("预览"), this);
	m_applyBtn = new QPushButton(QStringLiteral("应用"), this);
	m_resetBtn = new QPushButton(QStringLiteral("重置"), this);
	m_undoBtn = new QPushButton(QStringLiteral("撤销"), this);
	m_redoBtn = new QPushButton(QStringLiteral("重做"), this);
	m_saveTemplateBtn = new QPushButton(QStringLiteral("保存模板"), this);
	m_loadTemplateBtn = new QPushButton(QStringLiteral("加载模板"), this);
	actionRow->addWidget(m_previewBtn);
	actionRow->addWidget(m_applyBtn);
	actionRow->addWidget(m_resetBtn);
	actionRow->addWidget(m_undoBtn);
	actionRow->addWidget(m_redoBtn);
	actionRow->addWidget(m_saveTemplateBtn);
	actionRow->addWidget(m_loadTemplateBtn);
	actionRow->addStretch(1);
	root->addLayout(actionRow);

	refreshScopeKindCombo();
	rebuildPalette();

	connect(m_programCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &TrajectoryEditPageWidget::onProgramChanged);
	connect(m_groupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &TrajectoryEditPageWidget::onGroupChanged);
	connect(m_palette, &QListWidget::itemDoubleClicked, this, &TrajectoryEditPageWidget::onPaletteDoubleClicked);
	connect(m_pipeline, &TrajectoryPipelineListWidget::opsChanged, this, &TrajectoryEditPageWidget::syncSessionPipeline);
	connect(m_pipeline, &TrajectoryPipelineListWidget::selectedOpChanged, this, &TrajectoryEditPageWidget::onPipelineSelectionChanged);
	connect(m_previewBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onPreviewClicked);
	connect(m_applyBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onApplyClicked);
	connect(m_resetBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onResetClicked);
	connect(m_undoBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onUndoClicked);
	connect(m_redoBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onRedoClicked);
	connect(m_removeOpBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onRemovePipelineOpClicked);
	connect(m_moveUpBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onMovePipelineOpUpClicked);
	connect(m_moveDownBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onMovePipelineOpDownClicked);
	connect(m_saveTemplateBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onSaveTemplateClicked);
	connect(m_loadTemplateBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onLoadTemplateClicked);

	auto paramChanged = [this]() {
		if (!m_loadingParams)
		{
			refreshScopeFieldVisibility();
			applyParamsToSelectedOp();
		}
	};
	connect(m_scopeKindCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, paramChanged);
	connect(m_scopeGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, paramChanged);
	for (QDoubleSpinBox* spin :
		{ m_pointFromSpin, m_pointToSpin, m_dxSpin, m_dySpin, m_dzSpin, m_axisXSpin, m_axisYSpin, m_axisZSpin, m_angleSpin })
	{
		connect(spin, QOverload<double>::of(&QDoubleSpinBox::valueChanged), this, paramChanged);
	}
	setUseChinese(m_useChinese);
}

void TrajectoryEditPageWidget::refreshScopeKindCombo()
{
	if (!m_scopeKindCombo)
	{
		return;
	}
	const int prevKind = m_scopeKindCombo->currentData().toInt();
	const bool zh = m_useChinese;
	m_scopeKindCombo->blockSignals(true);
	m_scopeKindCombo->clear();
	m_scopeKindCombo->addItem(
		zh ? QStringLiteral("分组") : QStringLiteral("Group"),
		static_cast<int>(RobotInstruction::OpScope::Kind::Group));
	m_scopeKindCombo->addItem(
		zh ? QStringLiteral("全程序") : QStringLiteral("Entire program"),
		static_cast<int>(RobotInstruction::OpScope::Kind::EntireProgram));
	m_scopeKindCombo->addItem(
		zh ? QStringLiteral("P 范围") : QStringLiteral("P range"),
		static_cast<int>(RobotInstruction::OpScope::Kind::PointIndexRange));
	const int idx = m_scopeKindCombo->findData(prevKind);
	m_scopeKindCombo->setCurrentIndex(idx >= 0 ? idx : 0);
	m_scopeKindCombo->blockSignals(false);
}

void TrajectoryEditPageWidget::updateUiLabels()
{
	const bool zh = m_useChinese;
	if (m_programLabel)
	{
		m_programLabel->setText(zh ? QStringLiteral("程序") : QStringLiteral("Program"));
	}
	if (m_groupLabel)
	{
		m_groupLabel->setText(zh ? QStringLiteral("分组") : QStringLiteral("Group"));
	}
	if (m_paramGroupBox)
	{
		m_paramGroupBox->setTitle(zh ? QStringLiteral("参数") : QStringLiteral("Parameters"));
	}
	if (m_scopeKindFieldLabel)
	{
		m_scopeKindFieldLabel->setText(zh ? QStringLiteral("作用域") : QStringLiteral("Scope"));
	}
	if (m_scopeGroupFieldLabel)
	{
		m_scopeGroupFieldLabel->setText(zh ? QStringLiteral("分组") : QStringLiteral("Group"));
	}
	if (m_pointRangeFieldLabel)
	{
		m_pointRangeFieldLabel->setText(QStringLiteral("P"));
	}
	if (m_dxFieldLabel)
	{
		m_dxFieldLabel->setText(zh ? QStringLiteral("ΔX mm") : QStringLiteral("dX mm"));
	}
	if (m_dyFieldLabel)
	{
		m_dyFieldLabel->setText(zh ? QStringLiteral("ΔY mm") : QStringLiteral("dY mm"));
	}
	if (m_dzFieldLabel)
	{
		m_dzFieldLabel->setText(zh ? QStringLiteral("ΔZ mm") : QStringLiteral("dZ mm"));
	}
	if (m_axisXFieldLabel)
	{
		m_axisXFieldLabel->setText(zh ? QStringLiteral("轴 X") : QStringLiteral("Axis X"));
	}
	if (m_axisYFieldLabel)
	{
		m_axisYFieldLabel->setText(zh ? QStringLiteral("轴 Y") : QStringLiteral("Axis Y"));
	}
	if (m_axisZFieldLabel)
	{
		m_axisZFieldLabel->setText(zh ? QStringLiteral("轴 Z") : QStringLiteral("Axis Z"));
	}
	if (m_angleFieldLabel)
	{
		m_angleFieldLabel->setText(zh ? QStringLiteral("角度 °") : QStringLiteral("Angle deg"));
	}
	if (m_removeOpBtn)
	{
		m_removeOpBtn->setText(zh ? QStringLiteral("移除块") : QStringLiteral("Remove block"));
	}
	if (m_moveUpBtn)
	{
		m_moveUpBtn->setText(zh ? QStringLiteral("上移") : QStringLiteral("Up"));
	}
	if (m_moveDownBtn)
	{
		m_moveDownBtn->setText(zh ? QStringLiteral("下移") : QStringLiteral("Down"));
	}
	if (m_previewBtn)
	{
		m_previewBtn->setText(zh ? QStringLiteral("预览") : QStringLiteral("Preview"));
	}
	if (m_applyBtn)
	{
		m_applyBtn->setText(zh ? QStringLiteral("应用") : QStringLiteral("Apply"));
	}
	if (m_resetBtn)
	{
		m_resetBtn->setText(zh ? QStringLiteral("重置") : QStringLiteral("Reset"));
	}
	if (m_undoBtn)
	{
		m_undoBtn->setText(zh ? QStringLiteral("撤销") : QStringLiteral("Undo"));
	}
	if (m_redoBtn)
	{
		m_redoBtn->setText(zh ? QStringLiteral("重做") : QStringLiteral("Redo"));
	}
	if (m_saveTemplateBtn)
	{
		m_saveTemplateBtn->setText(zh ? QStringLiteral("保存模板") : QStringLiteral("Save template"));
	}
	if (m_loadTemplateBtn)
	{
		m_loadTemplateBtn->setText(zh ? QStringLiteral("加载模板") : QStringLiteral("Load template"));
	}
	refreshScopeKindCombo();
}

void TrajectoryEditPageWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	updateUiLabels();
	if (m_pipeline)
	{
		m_pipeline->setUseChinese(chinese);
		const std::vector<RobotInstruction::TrajectoryOpDescriptor> ops = m_pipeline->ops();
		m_pipeline->setOps(ops);
	}
	rebuildPalette();
	refreshProgramAndGroupCombos();
}

void TrajectoryEditPageWidget::setReadOnly(const bool readOnly)
{
	m_readOnly = readOnly;
	QWidget* widgets[] = {
		m_programCombo,
		m_groupCombo,
		m_palette,
		m_pipeline,
		m_scopeKindCombo,
		m_scopeGroupCombo,
		m_pointFromSpin,
		m_pointToSpin,
		m_dxSpin,
		m_dySpin,
		m_dzSpin,
		m_axisXSpin,
		m_axisYSpin,
		m_axisZSpin,
		m_angleSpin,
		m_previewBtn,
		m_applyBtn,
		m_resetBtn,
		m_undoBtn,
		m_redoBtn,
		m_removeOpBtn,
		m_moveUpBtn,
		m_moveDownBtn,
		m_saveTemplateBtn,
		m_loadTemplateBtn,
	};
	for (QWidget* w : widgets)
	{
		if (w)
		{
			w->setEnabled(!readOnly);
		}
	}
}

void TrajectoryEditPageWidget::bindStore(RobotProgramStore* store)
{
	m_store = store;
	refreshProgramAndGroupCombos();
}

void TrajectoryEditPageWidget::bindEditService(ProgramEditService* service)
{
	m_editService = service;
	if (m_editService)
	{
		connect(m_editService, &ProgramEditService::revisionChanged, this, [this](int) {
			syncUiAfterProgramRevision();
			if (m_commandPage)
			{
				m_commandPage->refreshInstructionList();
			}
		});
	}
	refreshUndoButtons();
}

void TrajectoryEditPageWidget::bindSession(TrajectoryEditSession* session)
{
	m_session = session;
}

void TrajectoryEditPageWidget::bindCommandPage(SimulationCommandWidget* commandPage)
{
	if (m_commandPage)
	{
		disconnect(m_commandPage, nullptr, this, nullptr);
	}
	m_commandPage = commandPage;
	if (!m_commandPage)
	{
		return;
	}
	connect(m_commandPage, &SimulationCommandWidget::activeProgramChanged, this, [this](const QString&) {
		refreshProgramAndGroupCombos();
	});
	connect(m_commandPage, &SimulationCommandWidget::groupsChanged, this, [this]() {
		refreshProgramAndGroupCombos();
		if (m_pipeline && m_pipeline->selectedOpIndex() >= 0)
		{
			loadSelectedOpToParams();
		}
	});
}

void TrajectoryEditPageWidget::refreshProgramAndGroupCombos()
{
	if (!m_store)
	{
		return;
	}
	const std::string activeId = m_store->activeProgramIdUtf8();
	const auto& catalog = m_store->activeCatalog();

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

	QString prevTopGroupId;
	if (m_groupCombo && m_groupCombo->currentIndex() > 0)
	{
		prevTopGroupId = m_groupCombo->currentData().toString();
	}
	else if (!m_selectedGroupId.empty())
	{
		prevTopGroupId = QString::fromStdString(m_selectedGroupId);
	}
	QString prevScopeGroupId;
	if (m_pipeline && m_pipeline->selectedOpIndex() >= 0)
	{
		const std::string opGroupId = m_pipeline->selectedOp().scope.groupId;
		if (!opGroupId.empty())
		{
			prevScopeGroupId = QString::fromStdString(opGroupId);
		}
	}
	if (prevScopeGroupId.isEmpty() && m_scopeGroupCombo && m_scopeGroupCombo->currentIndex() >= 0)
	{
		prevScopeGroupId = m_scopeGroupCombo->currentData().toString();
	}

	m_groupCombo->blockSignals(true);
	m_groupCombo->clear();
	m_groupCombo->addItem(m_useChinese ? QStringLiteral("（无）") : QStringLiteral("(none)"), QString());
	const RobotInstruction::RobotProgram* prog = catalog.findProgram(activeId);
	if (prog)
	{
		for (const RobotInstruction::InstructionGroup& group : prog->groups)
		{
			m_groupCombo->addItem(QString::fromStdString(group.name), QString::fromStdString(group.id));
		}
	}
	int topGroupIdx = 0;
	if (!prevTopGroupId.isEmpty())
	{
		const int gIdx = m_groupCombo->findData(prevTopGroupId);
		if (gIdx >= 0)
		{
			topGroupIdx = gIdx;
		}
	}
	m_groupCombo->setCurrentIndex(topGroupIdx);
	m_groupCombo->blockSignals(false);
	if (topGroupIdx > 0)
	{
		m_selectedGroupId = prevTopGroupId.toStdString();
	}
	else
	{
		m_selectedGroupId.clear();
	}

	m_scopeGroupCombo->blockSignals(true);
	m_scopeGroupCombo->clear();
	if (prog)
	{
		for (const RobotInstruction::InstructionGroup& group : prog->groups)
		{
			m_scopeGroupCombo->addItem(QString::fromStdString(group.name), QString::fromStdString(group.id));
		}
	}
	if (!prevScopeGroupId.isEmpty())
	{
		const int scopeIdx = m_scopeGroupCombo->findData(prevScopeGroupId);
		if (scopeIdx >= 0)
		{
			m_scopeGroupCombo->setCurrentIndex(scopeIdx);
		}
	}
	m_scopeGroupCombo->blockSignals(false);

	if (m_session)
	{
		m_session->setContextProgramId(activeId);
	}
}

void TrajectoryEditPageWidget::rebuildPalette()
{
	m_palette->clear();
	const RobotInstruction::TrajectoryOpKind kinds[] = {
		RobotInstruction::TrajectoryOpKind::Translate,
		RobotInstruction::TrajectoryOpKind::Rotate,
		RobotInstruction::TrajectoryOpKind::Delete,
		RobotInstruction::TrajectoryOpKind::Duplicate,
		RobotInstruction::TrajectoryOpKind::Mirror,
		RobotInstruction::TrajectoryOpKind::Reorder,
	};
	for (const RobotInstruction::TrajectoryOpKind kind : kinds)
	{
		auto* item = new QListWidgetItem(opKindLabel(kind, m_useChinese), m_palette);
		item->setData(Qt::UserRole, static_cast<int>(kind));
	}
}

RobotInstruction::OpScope TrajectoryEditPageWidget::defaultScopeForNewOp() const
{
	RobotInstruction::OpScope scope{};
	if (m_groupCombo && m_groupCombo->currentIndex() > 0)
	{
		scope.kind = RobotInstruction::OpScope::Kind::Group;
		scope.groupId = m_groupCombo->currentData().toString().toStdString();
	}
	else
	{
		scope.kind = RobotInstruction::OpScope::Kind::EntireProgram;
	}
	return scope;
}

void TrajectoryEditPageWidget::syncScopeGroupFromTopBar()
{
	if (!m_groupCombo || !m_scopeGroupCombo || m_groupCombo->currentIndex() <= 0)
	{
		return;
	}
	const QString groupId = m_groupCombo->currentData().toString();
	const int gIdx = m_scopeGroupCombo->findData(groupId);
	if (gIdx >= 0)
	{
		m_scopeGroupCombo->setCurrentIndex(gIdx);
	}
}

void TrajectoryEditPageWidget::fillScopeFromUi(RobotInstruction::OpScope& scope) const
{
	scope.kind = static_cast<RobotInstruction::OpScope::Kind>(m_scopeKindCombo->currentData().toInt());
	scope.instructionIds.clear();
	scope.groupId.clear();
	scope.pointFrom = static_cast<int>(m_pointFromSpin->value());
	scope.pointTo = static_cast<int>(m_pointToSpin->value());
	if (scope.kind == RobotInstruction::OpScope::Kind::Group)
	{
		if (m_scopeGroupCombo && m_scopeGroupCombo->currentIndex() >= 0)
		{
			scope.groupId = m_scopeGroupCombo->currentData().toString().toStdString();
		}
		if (scope.groupId.empty() && m_groupCombo && m_groupCombo->currentIndex() > 0)
		{
			scope.groupId = m_groupCombo->currentData().toString().toStdString();
		}
	}
}

RobotInstruction::TrajectoryOpDescriptor TrajectoryEditPageWidget::makeDefaultOp(
	const RobotInstruction::TrajectoryOpKind kind) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = kind;
	op.scope = defaultScopeForNewOp();
	return op;
}

void TrajectoryEditPageWidget::syncSessionPipeline()
{
	if (!m_session || !m_pipeline)
	{
		return;
	}
	m_session->setPipeline(m_pipeline->ops());
}

void TrajectoryEditPageWidget::syncSessionParams()
{
	if (!m_session || !m_pipeline)
	{
		return;
	}
	m_session->updatePipelineOps(m_pipeline->ops());
}

void TrajectoryEditPageWidget::flushPipelineToSession()
{
	if (!m_session || !m_pipeline)
	{
		return;
	}
	// 拖放 bug 可能导致列表有项但 m_ops 为空，强制与数据对齐
	if (m_pipeline->count() != static_cast<int>(m_pipeline->ops().size()))
	{
		m_pipeline->setOps(m_pipeline->ops());
	}
	if (m_pipeline->selectedOpIndex() >= 0)
	{
		applyParamsToSelectedOp();
	}
	else
	{
		syncSessionParams();
	}
}

void TrajectoryEditPageWidget::refreshScopeFieldVisibility()
{
	if (!m_scopeKindCombo)
	{
		return;
	}
	const auto scopeKind = static_cast<RobotInstruction::OpScope::Kind>(m_scopeKindCombo->currentData().toInt());
	const bool showGroup = scopeKind == RobotInstruction::OpScope::Kind::Group;
	const bool showRange = scopeKind == RobotInstruction::OpScope::Kind::PointIndexRange;
	if (m_scopeGroupFieldLabel)
	{
		m_scopeGroupFieldLabel->setVisible(showGroup);
	}
	if (m_scopeGroupCombo)
	{
		m_scopeGroupCombo->setVisible(showGroup);
	}
	if (m_pointRangeFieldLabel)
	{
		m_pointRangeFieldLabel->setVisible(showRange);
	}
	if (m_pointFromSpin)
	{
		m_pointFromSpin->setVisible(showRange);
	}
	if (m_pointToSpin)
	{
		m_pointToSpin->setVisible(showRange);
	}
}

void TrajectoryEditPageWidget::refreshParamPanelForKind(const RobotInstruction::TrajectoryOpKind kind)
{
	const bool showTranslate = kind == RobotInstruction::TrajectoryOpKind::Translate;
	const bool showRotate = kind == RobotInstruction::TrajectoryOpKind::Rotate;
	const bool showMirrorHint = kind == RobotInstruction::TrajectoryOpKind::Mirror;
	const auto setRowVisible = [](QLabel* label, QWidget* widget, const bool visible) {
		if (label)
		{
			label->setVisible(visible);
		}
		if (widget)
		{
			widget->setVisible(visible);
		}
	};
	setRowVisible(m_dxFieldLabel, m_dxSpin, showTranslate);
	setRowVisible(m_dyFieldLabel, m_dySpin, showTranslate);
	setRowVisible(m_dzFieldLabel, m_dzSpin, showTranslate);
	setRowVisible(m_axisXFieldLabel, m_axisXSpin, showRotate);
	setRowVisible(m_axisYFieldLabel, m_axisYSpin, showRotate);
	setRowVisible(m_axisZFieldLabel, m_axisZSpin, showRotate);
	setRowVisible(m_angleFieldLabel, m_angleSpin, showRotate);
	if (m_mirrorHintLabel)
	{
		m_mirrorHintLabel->setVisible(showMirrorHint);
		m_mirrorHintLabel->setText(
			m_useChinese ? QStringLiteral("镜像块尚未实现，预览与应用暂不可用。")
						 : QStringLiteral("Mirror block is not implemented yet."));
	}
	refreshScopeFieldVisibility();
}

void TrajectoryEditPageWidget::loadSelectedOpToParams()
{
	if (!m_pipeline)
	{
		return;
	}
	const RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->selectedOp();
	m_loadingParams = true;
	const int scopeIdx = m_scopeKindCombo->findData(static_cast<int>(op.scope.kind));
	if (scopeIdx >= 0)
	{
		m_scopeKindCombo->setCurrentIndex(scopeIdx);
	}
	else if (op.scope.kind == RobotInstruction::OpScope::Kind::InstructionIds)
	{
		m_scopeKindCombo->setCurrentIndex(m_scopeKindCombo->findData(
			static_cast<int>(RobotInstruction::OpScope::Kind::Group)));
	}
	if (!op.scope.groupId.empty())
	{
		const int gIdx = m_scopeGroupCombo->findData(QString::fromStdString(op.scope.groupId));
		if (gIdx >= 0)
		{
			m_scopeGroupCombo->setCurrentIndex(gIdx);
		}
	}
	m_pointFromSpin->setValue(op.scope.pointFrom);
	m_pointToSpin->setValue(op.scope.pointTo);
	m_dxSpin->setValue(op.translate.dxMm);
	m_dySpin->setValue(op.translate.dyMm);
	m_dzSpin->setValue(op.translate.dzMm);
	m_axisXSpin->setValue(op.rotate.axisX);
	m_axisYSpin->setValue(op.rotate.axisY);
	m_axisZSpin->setValue(op.rotate.axisZ);
	m_angleSpin->setValue(op.rotate.angleDeg);
	m_loadingParams = false;
	refreshParamPanelForKind(op.kind);
}

void TrajectoryEditPageWidget::applyParamsToSelectedOp()
{
	if (!m_pipeline || m_pipeline->selectedOpIndex() < 0)
	{
		return;
	}
	RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->selectedOp();
	const std::string storedGroupId = op.scope.groupId;
	fillScopeFromUi(op.scope);
	if (op.scope.kind == RobotInstruction::OpScope::Kind::Group
		&& op.scope.groupId.empty()
		&& !storedGroupId.empty()
		&& m_store)
	{
		const RobotInstruction::RobotProgram* prog =
			m_store->activeCatalog().findProgram(m_store->activeProgramIdUtf8());
		if (prog)
		{
			for (const RobotInstruction::InstructionGroup& group : prog->groups)
			{
				if (group.id == storedGroupId)
				{
					op.scope.groupId = storedGroupId;
					break;
				}
			}
		}
	}
	op.translate.dxMm = m_dxSpin->value();
	op.translate.dyMm = m_dySpin->value();
	op.translate.dzMm = m_dzSpin->value();
	op.rotate.axisX = m_axisXSpin->value();
	op.rotate.axisY = m_axisYSpin->value();
	op.rotate.axisZ = m_axisZSpin->value();
	op.rotate.angleDeg = m_angleSpin->value();
	m_pipeline->updateSelectedOp(op);
	syncSessionParams();
}

void TrajectoryEditPageWidget::refreshUndoButtons()
{
	if (m_undoBtn && m_editService)
	{
		m_undoBtn->setEnabled(m_editService->canUndo() && !m_readOnly);
	}
	if (m_redoBtn && m_editService)
	{
		m_redoBtn->setEnabled(m_editService->canRedo() && !m_readOnly);
	}
}

void TrajectoryEditPageWidget::reconcilePipelineScopes()
{
	if (!m_store || !m_pipeline)
	{
		return;
	}
	const RobotInstruction::RobotProgram* prog =
		m_store->activeCatalog().findProgram(m_store->activeProgramIdUtf8());
	if (!prog)
	{
		return;
	}
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops = m_pipeline->ops();
	bool changed = false;
	for (RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		if (op.scope.kind != RobotInstruction::OpScope::Kind::Group || op.scope.groupId.empty())
		{
			continue;
		}
		const RobotInstruction::InstructionGroup* matched = nullptr;
		for (const RobotInstruction::InstructionGroup& group : prog->groups)
		{
			if (group.id == op.scope.groupId)
			{
				matched = &group;
				break;
			}
		}
		if (matched && !matched->memberInstructionIds.empty())
		{
			continue;
		}
		std::string fallbackId;
		if (!m_selectedGroupId.empty())
		{
			for (const RobotInstruction::InstructionGroup& group : prog->groups)
			{
				if (group.id == m_selectedGroupId && !group.memberInstructionIds.empty())
				{
					fallbackId = group.id;
					break;
				}
			}
		}
		if (!fallbackId.empty())
		{
			op.scope.groupId = fallbackId;
		}
		else
		{
			op.scope.kind = RobotInstruction::OpScope::Kind::EntireProgram;
			op.scope.groupId.clear();
		}
		changed = true;
	}
	if (changed)
	{
		m_pipeline->setOps(ops);
	}
}

void TrajectoryEditPageWidget::syncUiAfterProgramRevision()
{
	refreshUndoButtons();
	refreshProgramAndGroupCombos();
	if (m_session)
	{
		m_session->abandonPreview();
		if (m_store)
		{
			m_session->setContextProgramId(m_store->activeProgramIdUtf8());
		}
	}
	reconcilePipelineScopes();
	if (m_pipeline && m_pipeline->selectedOpIndex() >= 0)
	{
		loadSelectedOpToParams();
	}
}

void TrajectoryEditPageWidget::onProgramChanged(int index)
{
	(void)index;
	if (!m_store || !m_programCombo)
	{
		return;
	}
	const std::string programId = m_programCombo->currentData().toString().toStdString();
	m_store->setActiveProgramIdUtf8(programId);
	if (m_session)
	{
		m_session->reset();
		m_session->setContextProgramId(programId);
	}
	if (m_commandPage)
	{
		m_commandPage->refreshInstructionList();
	}
	refreshProgramAndGroupCombos();
}

void TrajectoryEditPageWidget::onGroupChanged(int index)
{
	(void)index;
	if (m_groupCombo && m_groupCombo->currentIndex() > 0)
	{
		m_selectedGroupId = m_groupCombo->currentData().toString().toStdString();
		syncScopeGroupFromTopBar();
		if (!m_loadingParams && m_pipeline && m_pipeline->selectedOpIndex() >= 0)
		{
			const auto kind = static_cast<RobotInstruction::OpScope::Kind>(m_scopeKindCombo->currentData().toInt());
			if (kind == RobotInstruction::OpScope::Kind::Group)
			{
				applyParamsToSelectedOp();
			}
		}
	}
	else
	{
		m_selectedGroupId.clear();
	}
	if (m_session)
	{
		m_session->setDefaultGroupId(m_selectedGroupId);
	}
}

void TrajectoryEditPageWidget::onPaletteDoubleClicked(QListWidgetItem* item)
{
	if (!item || !m_pipeline || m_readOnly)
	{
		return;
	}
	const auto kind = static_cast<RobotInstruction::TrajectoryOpKind>(item->data(Qt::UserRole).toInt());
	m_pipeline->appendOp(makeDefaultOp(kind));
	syncScopeGroupFromTopBar();
	loadSelectedOpToParams();
	syncSessionPipeline();
}

void TrajectoryEditPageWidget::onPipelineSelectionChanged(int index)
{
	(void)index;
	if (!m_pipeline || m_pipeline->selectedOpIndex() < 0)
	{
		return;
	}
	loadSelectedOpToParams();
}

void TrajectoryEditPageWidget::onPreviewClicked()
{
	if (!m_session)
	{
		return;
	}
	reconcilePipelineScopes();
	flushPipelineToSession();
	QString err;
	if (!m_session->preview(&err))
	{
		QMessageBox::warning(
			this,
			m_useChinese ? QStringLiteral("预览") : QStringLiteral("Preview"),
			err);
	}
}

void TrajectoryEditPageWidget::onApplyClicked()
{
	if (!m_session)
	{
		return;
	}
	reconcilePipelineScopes();
	flushPipelineToSession();
	QString err;
	if (!m_session->apply(&err))
	{
		QMessageBox::warning(
			this,
			m_useChinese ? QStringLiteral("应用") : QStringLiteral("Apply"),
			err);
		return;
	}
	if (m_commandPage)
	{
		m_commandPage->refreshInstructionList();
	}
	refreshUndoButtons();
}

void TrajectoryEditPageWidget::onResetClicked()
{
	if (m_session)
	{
		m_session->reset();
	}
	if (m_pipeline)
	{
		m_pipeline->setOps({});
	}
	if (m_pipeline && m_pipeline->selectedOpIndex() >= 0)
	{
		loadSelectedOpToParams();
	}
}

void TrajectoryEditPageWidget::onUndoClicked()
{
	if (!m_editService)
	{
		return;
	}
	QString err;
	if (!m_editService->undo(&err))
	{
		QMessageBox::warning(
			this,
			m_useChinese ? QStringLiteral("撤销") : QStringLiteral("Undo"),
			err);
	}
}

void TrajectoryEditPageWidget::onRedoClicked()
{
	if (!m_editService)
	{
		return;
	}
	QString err;
	if (!m_editService->redo(&err))
	{
		QMessageBox::warning(
			this,
			m_useChinese ? QStringLiteral("重做") : QStringLiteral("Redo"),
			err);
	}
}

void TrajectoryEditPageWidget::onRemovePipelineOpClicked()
{
	if (m_pipeline)
	{
		m_pipeline->removeSelectedOp();
		syncSessionPipeline();
	}
}

void TrajectoryEditPageWidget::onMovePipelineOpUpClicked()
{
	if (m_pipeline)
	{
		m_pipeline->moveSelectedOp(-1);
		syncSessionPipeline();
	}
}

void TrajectoryEditPageWidget::onMovePipelineOpDownClicked()
{
	if (m_pipeline)
	{
		m_pipeline->moveSelectedOp(1);
		syncSessionPipeline();
	}
}

void TrajectoryEditPageWidget::onSaveTemplateClicked()
{
	if (!m_pipeline)
	{
		return;
	}
	QSettings settings(QStringLiteral("CloudSim"), QStringLiteral("TrajectoryPipeline"));
	settings.setValue(QStringLiteral("opsCount"), static_cast<int>(m_pipeline->ops().size()));
	int i = 0;
	for (const RobotInstruction::TrajectoryOpDescriptor& op : m_pipeline->ops())
	{
		const QString prefix = QStringLiteral("op/%1/").arg(i++);
		settings.setValue(prefix + QStringLiteral("kind"), static_cast<int>(op.kind));
		settings.setValue(prefix + QStringLiteral("scopeKind"), static_cast<int>(op.scope.kind));
		settings.setValue(prefix + QStringLiteral("groupId"), QString::fromStdString(op.scope.groupId));
		settings.setValue(prefix + QStringLiteral("pointFrom"), op.scope.pointFrom);
		settings.setValue(prefix + QStringLiteral("pointTo"), op.scope.pointTo);
		settings.setValue(prefix + QStringLiteral("dx"), op.translate.dxMm);
		settings.setValue(prefix + QStringLiteral("dy"), op.translate.dyMm);
		settings.setValue(prefix + QStringLiteral("dz"), op.translate.dzMm);
		settings.setValue(prefix + QStringLiteral("ax"), op.rotate.axisX);
		settings.setValue(prefix + QStringLiteral("ay"), op.rotate.axisY);
		settings.setValue(prefix + QStringLiteral("az"), op.rotate.axisZ);
		settings.setValue(prefix + QStringLiteral("angle"), op.rotate.angleDeg);
	}
}

void TrajectoryEditPageWidget::onLoadTemplateClicked()
{
	QSettings settings(QStringLiteral("CloudSim"), QStringLiteral("TrajectoryPipeline"));
	const int count = settings.value(QStringLiteral("opsCount"), 0).toInt();
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops;
	ops.reserve(static_cast<size_t>(count));
	for (int i = 0; i < count; ++i)
	{
		const QString prefix = QStringLiteral("op/%1/").arg(i);
		RobotInstruction::TrajectoryOpDescriptor op{};
		op.kind = static_cast<RobotInstruction::TrajectoryOpKind>(settings.value(prefix + QStringLiteral("kind"), 0).toInt());
		op.scope.kind = static_cast<RobotInstruction::OpScope::Kind>(
			settings.value(prefix + QStringLiteral("scopeKind"), 0).toInt());
		op.scope.groupId = settings.value(prefix + QStringLiteral("groupId")).toString().toStdString();
		op.scope.pointFrom = settings.value(prefix + QStringLiteral("pointFrom"), 1).toInt();
		op.scope.pointTo = settings.value(prefix + QStringLiteral("pointTo"), 1).toInt();
		op.translate.dxMm = settings.value(prefix + QStringLiteral("dx"), 0.0).toDouble();
		op.translate.dyMm = settings.value(prefix + QStringLiteral("dy"), 0.0).toDouble();
		op.translate.dzMm = settings.value(prefix + QStringLiteral("dz"), 0.0).toDouble();
		op.rotate.axisX = settings.value(prefix + QStringLiteral("ax"), 0.0).toDouble();
		op.rotate.axisY = settings.value(prefix + QStringLiteral("ay"), 0.0).toDouble();
		op.rotate.axisZ = settings.value(prefix + QStringLiteral("az"), 1.0).toDouble();
		op.rotate.angleDeg = settings.value(prefix + QStringLiteral("angle"), 0.0).toDouble();
		ops.push_back(op);
	}
	if (m_pipeline)
	{
		m_pipeline->setOps(ops);
		syncSessionPipeline();
	}
}
