#include "TrajectoryEditPageWidget.h"

#include "FeaturePickTransform.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "ProgramEditService.h"
#include "RobotOsgUiTypes.h"
#include "RobotSimulationController.h"
#include "RawTrajectory.h"
#include "SimulationCommandWidget.h"
#include "TrajectoryEditSession.h"
#include "TrajectoryOpParamPanel.h"
#include "TrajectoryPipelineListWidget.h"
#include "RobotProgramStore.h"

#include <ITrajectoryOp.h>
#include "TrajectoryOpBridge.h"

#include <json.hpp>

#include <QCheckBox>
#include <QComboBox>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QSignalBlocker>
#include <QTimer>
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
		setSpacing(2);
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
void updateTransformActionButtons(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	QCheckBox* previewCheck,
	QPushButton* applyBtn,
	const bool readOnly)
{
	const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(op.kind);
	const bool canTransform = algo
		&& (trajectory_algo::hasCapability(
				algo->capabilities(),
				trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform)
			|| trajectory_algo::hasCapability(
				algo->capabilities(),
				trajectory_algo::TrajectoryOpCapability::ApplyPoseTransform)
			|| trajectory_algo::hasCapability(
				algo->capabilities(),
				trajectory_algo::TrajectoryOpCapability::ApplyStructuralEdit));
	if (previewCheck)
	{
		previewCheck->setEnabled(canTransform && !readOnly
			&& algo
			&& trajectory_algo::hasCapability(
				algo->capabilities(),
				trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform));
	}
	if (applyBtn)
	{
		applyBtn->setEnabled(canTransform && !readOnly);
	}
}

} // namespace

TrajectoryEditPageWidget::TrajectoryEditPageWidget(QWidget* parent)
	: QWidget(parent)
{
	RobotInstruction::ensureTrajectoryOpBuiltinsRegistered();

	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(6, 6, 6, 6);
	root->setSpacing(6);

	m_rawGroupBox = new QGroupBox(QStringLiteral("原始轨迹"), this);
	auto* rawLayout = new QVBoxLayout(m_rawGroupBox);
	m_rawStatusLabel = new QLabel(this);
	rawLayout->addWidget(m_rawStatusLabel);
	m_rawRecipeCombo = new QComboBox(m_rawGroupBox);
	m_rawRecipeCombo->addItem(QStringLiteral("焊缝默认"), QStringLiteral("weld"));
	m_rawRecipeCombo->addItem(QStringLiteral("涂胶默认"), QStringLiteral("glue"));
	m_rawRecipeCombo->addItem(QStringLiteral("打磨默认"), QStringLiteral("grind"));
	rawLayout->addWidget(m_rawRecipeCombo);
	m_rawApplyBtn = new QPushButton(QStringLiteral("应用配方流水线"), m_rawGroupBox);
	m_rawEmitBtn = new QPushButton(QStringLiteral("生成程序"), m_rawGroupBox);
	rawLayout->addWidget(m_rawApplyBtn);
	rawLayout->addWidget(m_rawEmitBtn);
	root->addWidget(m_rawGroupBox);

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
	auto* paramBoxLayout = new QVBoxLayout(m_paramGroupBox);
	m_scopeGroupCombo = new QComboBox(m_paramGroupBox);
	m_paramPanel = new TrajectoryOpParamPanel(m_paramGroupBox);
	m_paramPanel->setScopeGroupCombo(m_scopeGroupCombo);
	paramBoxLayout->addWidget(m_paramPanel);
	root->addWidget(m_paramGroupBox);

	m_pipeline->setContextMenuPolicy(Qt::CustomContextMenu);

	auto* actionRow = new QHBoxLayout;
	m_previewCheck = new QCheckBox(QStringLiteral("预览"), this);
	m_previewCheck->setChecked(true);
	m_applyBtn = new QPushButton(QStringLiteral("应用"), this);
	m_resetBtn = new QPushButton(QStringLiteral("重置"), this);
	m_undoBtn = new QPushButton(QStringLiteral("撤销"), this);
	m_redoBtn = new QPushButton(QStringLiteral("重做"), this);
	m_saveTemplateBtn = new QPushButton(QStringLiteral("保存模板"), this);
	m_loadTemplateBtn = new QPushButton(QStringLiteral("加载模板"), this);
	actionRow->addWidget(m_previewCheck);
	actionRow->addWidget(m_applyBtn);
	actionRow->addWidget(m_resetBtn);
	actionRow->addWidget(m_undoBtn);
	actionRow->addWidget(m_redoBtn);
	actionRow->addWidget(m_saveTemplateBtn);
	actionRow->addWidget(m_loadTemplateBtn);
	actionRow->addStretch(1);
	root->addLayout(actionRow);

	rebuildPalette();

	connect(m_programCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &TrajectoryEditPageWidget::onProgramChanged);
	connect(m_groupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &TrajectoryEditPageWidget::onGroupChanged);
	connect(m_palette, &QListWidget::itemDoubleClicked, this, &TrajectoryEditPageWidget::onPaletteDoubleClicked);
	connect(m_pipeline, &TrajectoryPipelineListWidget::opsChanged, this, [this]() {
		syncSessionPipeline();
		if (m_loadingParams || (m_paramPanel && m_paramPanel->isRebuilding()))
		{
			return;
		}
		QTimer::singleShot(0, this, [this]() {
			if (m_loadingParams || (m_paramPanel && m_paramPanel->isRebuilding()))
			{
				return;
			}
			runPreviewIfEnabled();
		});
	});
	connect(m_pipeline, &TrajectoryPipelineListWidget::selectedOpChanged, this, &TrajectoryEditPageWidget::onPipelineSelectionChanged);
	connect(
		m_pipeline,
		&QWidget::customContextMenuRequested,
		this,
		&TrajectoryEditPageWidget::showPipelineContextMenu);
	connect(m_previewCheck, &QCheckBox::toggled, this, &TrajectoryEditPageWidget::onPreviewToggled);
	connect(m_applyBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onApplyClicked);
	connect(m_resetBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onResetClicked);
	connect(m_undoBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onUndoClicked);
	connect(m_redoBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onRedoClicked);
	connect(m_saveTemplateBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onSaveTemplateClicked);
	connect(m_loadTemplateBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onLoadTemplateClicked);
	connect(m_rawApplyBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onRawApplyRecipe);
	connect(m_rawEmitBtn, &QPushButton::clicked, this, &TrajectoryEditPageWidget::onRawEmitProgram);

	connect(m_paramPanel, &TrajectoryOpParamPanel::paramsChanged, this, [this]() {
		if (!m_loadingParams)
		{
			applyParamsToSelectedOp();
			if (!m_flushingParams && m_previewCheck && m_previewCheck->isChecked()
				&& m_session && !m_session->isPreviewActive())
			{
				runPreviewIfEnabled();
			}
		}
	});
	connect(m_scopeGroupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, [this]() {
		if (!m_loadingParams)
		{
			applyParamsToSelectedOp();
			if (!m_flushingParams && m_previewCheck && m_previewCheck->isChecked()
				&& m_session && !m_session->isPreviewActive())
			{
				runPreviewIfEnabled();
			}
		}
	});
	setUseChinese(m_useChinese);
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
	if (m_paramPanel)
	{
		m_paramPanel->setUseChinese(zh);
	}
	if (m_previewCheck)
	{
		m_previewCheck->setText(zh ? QStringLiteral("预览") : QStringLiteral("Preview"));
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
	if (m_rawGroupBox)
	{
		m_rawGroupBox->setTitle(zh ? QStringLiteral("原始轨迹") : QStringLiteral("Raw trajectory"));
	}
	if (m_rawApplyBtn)
	{
		m_rawApplyBtn->setText(zh ? QStringLiteral("应用配方流水线") : QStringLiteral("Apply recipe pipeline"));
	}
	if (m_rawEmitBtn)
	{
		m_rawEmitBtn->setText(zh ? QStringLiteral("生成程序") : QStringLiteral("Emit program"));
	}
	refreshRawTrajectoryStatus();
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
		m_scopeGroupCombo,
		m_paramPanel,
		m_previewCheck,
		m_applyBtn,
		m_resetBtn,
		m_undoBtn,
		m_redoBtn,
		m_saveTemplateBtn,
		m_loadTemplateBtn,
		m_rawRecipeCombo,
	};
	for (QWidget* w : widgets)
	{
		if (w)
		{
			w->setEnabled(!readOnly);
		}
	}
	refreshRawTrajectoryStatus();
}

void TrajectoryEditPageWidget::bindStore(RobotProgramStore* store)
{
	m_store = store;
	refreshProgramAndGroupCombos();
	refreshRawTrajectoryStatus();
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
	if (m_session)
	{
		disconnect(m_session, nullptr, this, nullptr);
	}
	m_session = session;
	if (m_session)
	{
		connect(m_session, &TrajectoryEditSession::rawTrajectoryChanged, this, [this]() {
			refreshRawTrajectoryStatus();
		});
	}
	refreshRawTrajectoryStatus();
}

void TrajectoryEditPageWidget::bindSimulationController(RobotSimulationController* controller)
{
	m_simController = controller;
}

void TrajectoryEditPageWidget::bindHost(IRobotMainWindowHost* host)
{
	m_host = host;
}

void TrajectoryEditPageWidget::refreshRawTrajectoryStatus()
{
	const bool zh = m_useChinese;
	if (!m_rawStatusLabel)
	{
		return;
	}
	if (!m_session || !m_session->hasRawTrajectory())
	{
		m_rawStatusLabel->setText(zh ? QStringLiteral("请先在轨迹生成页离散")
			: QStringLiteral("Discretize on Trajectory Generation tab first"));
		if (m_rawApplyBtn)
		{
			m_rawApplyBtn->setEnabled(false);
		}
		if (m_rawEmitBtn)
		{
			m_rawEmitBtn->setEnabled(false);
		}
		return;
	}
	const RobotInstruction::RawTrajectory* traj = m_session->rawTrajectory();
	const int n = traj ? static_cast<int>(traj->points.size()) : 0;
	m_rawStatusLabel->setText(zh ? QStringLiteral("原始轨迹：%1 点").arg(n)
		: QStringLiteral("Raw trajectory: %1 points").arg(n));
	if (m_rawApplyBtn)
	{
		m_rawApplyBtn->setEnabled(!m_readOnly);
	}
	if (m_rawEmitBtn)
	{
		m_rawEmitBtn->setEnabled(!m_readOnly && m_store != nullptr);
	}
}

std::string TrajectoryEditPageWidget::resolvePreviewBackendId(const RobotInstruction::RawTrajectory& traj) const
{
	if (!traj.sourceFeature.workpiece.backendIdUtf8.empty())
	{
		return traj.sourceFeature.workpiece.backendIdUtf8;
	}
	return {};
}

void TrajectoryEditPageWidget::showRawTrajectoryPreview(const RobotInstruction::RawTrajectory& traj)
{
	if (!m_host)
	{
		return;
	}
	IRobotOsgViewHost* osg = m_host->osgView();
	if (!osg)
	{
		return;
	}
	const std::string backendId = resolvePreviewBackendId(traj);
	if (backendId.empty())
	{
		if (m_host)
		{
			m_host->appendRunWarning(m_useChinese ? QStringLiteral("轨迹预览：FeatureSpec 缺少 workpiece.backendIdUtf8")
				: QStringLiteral("Trajectory preview: missing workpiece.backendIdUtf8"));
		}
		return;
	}
	if (traj.points.empty())
	{
		return;
	}
	RobotOsgUi::RawTrajectoryPreviewOptions options;
	options.showAxes = true;
	options.axisInterval = 0;
	options.maxAxes = 50;
	std::string err;
	feature_pick_transform::applyRawTrajectoryPreviewToOsg(osg, backendId, traj, options, &err);
	if (!err.empty())
	{
		if (m_host)
		{
			m_host->appendRunWarning(QString::fromStdString(err));
		}
		if (m_simController)
		{
			m_simController->setRawTrajectoryPreviewActive(false);
		}
		return;
	}
	if (m_simController)
	{
		m_simController->setRawTrajectoryPreviewActive(true);
	}
	osg->requestRedraw();
}

void TrajectoryEditPageWidget::onRawApplyRecipe()
{
	if (!m_session || !m_session->hasRawTrajectory())
	{
		return;
	}
	const RobotInstruction::RawTrajectory* src = m_session->rawTrajectory();
	if (!src)
	{
		return;
	}
	RobotInstruction::RawTrajectory traj = *src;
	const QString recipe = m_rawRecipeCombo->currentData().toString();
	std::vector<RobotInstruction::RawTrajectoryOpDescriptor> ops;
	if (recipe == QStringLiteral("glue"))
	{
		ops = RobotInstruction::rawTrajectoryRecipeGlueDefault();
	}
	else if (recipe == QStringLiteral("grind"))
	{
		ops = RobotInstruction::rawTrajectoryRecipeGrindDefault();
	}
	else
	{
		ops = RobotInstruction::rawTrajectoryRecipeWeldDefault();
	}
	std::string err;
	if (!RobotInstruction::applyRawTrajectoryPipeline(ops, traj, &err))
	{
		QMessageBox::warning(this, QStringLiteral("配方"), QString::fromStdString(err));
		return;
	}
	m_session->setRawTrajectory(traj);
	showRawTrajectoryPreview(traj);
	if (m_host)
	{
		m_host->appendRunInfo(m_useChinese ? QStringLiteral("配方已应用")
			: QStringLiteral("Recipe applied"));
	}
}

void TrajectoryEditPageWidget::onRawEmitProgram()
{
	if (!m_session || !m_store || !m_session->hasRawTrajectory())
	{
		return;
	}
	const RobotInstruction::RawTrajectory* src = m_session->rawTrajectory();
	if (!src)
	{
		return;
	}
	RobotInstruction::RobotProgram* prog = m_store->activeCatalog().mainProgram();
	if (!prog)
	{
		return;
	}
	const std::string backendId = resolvePreviewBackendId(*src);
	if (backendId.empty())
	{
		QMessageBox::warning(this, QStringLiteral("生成"),
			m_useChinese ? QStringLiteral("FeatureSpec 缺少 workpiece.backendIdUtf8")
				: QStringLiteral("FeatureSpec missing workpiece.backendIdUtf8"));
		return;
	}
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	if (!osg)
	{
		return;
	}
	RobotInstruction::RawTrajectory worldTraj;
	std::string err;
	std::string emittedGroupId;
	if (!feature_pick_transform::transformRawTrajectoryToWorld(osg, backendId, *src, worldTraj, &err))
	{
		QMessageBox::warning(this, QStringLiteral("生成"), QString::fromStdString(err));
		return;
	}
	if (!RobotInstruction::emitRawTrajectoryToProgram(worldTraj, *prog, &err, &emittedGroupId))
	{
		QMessageBox::warning(this, QStringLiteral("生成"), QString::fromStdString(err));
		return;
	}
	if (m_simController)
	{
		m_simController->setRawTrajectoryPreviewActive(false);
	}
	if (m_commandPage)
	{
		m_commandPage->refreshInstructionList();
	}
	if (!emittedGroupId.empty())
	{
		m_selectedGroupId = emittedGroupId;
	}
	refreshProgramAndGroupCombos();
	if (m_simController)
	{
		m_simController->refreshInstructionPoseAxes(false);
	}
	if (osg)
	{
		osg->clearRawTrajectoryOverlay();
		osg->clearRawTrajectoryOverlayFrames();
		osg->requestRedraw();
	}
	if (m_host)
	{
		const QString groupName = src->sourceFeature.featureId.empty()
			? QStringLiteral("RawTrajectory")
			: QString::fromStdString(src->sourceFeature.featureId);
		m_host->appendRunInfo(m_useChinese
			? QStringLiteral("已写入主程序，分组「%1」").arg(groupName)
			: QStringLiteral("Written to main program, group \"%1\"").arg(groupName));
	}
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
	const std::vector<RobotInstruction::TrajectoryOpKind> kinds =
		RobotInstruction::trajectoryOpPaletteKinds();
	for (const RobotInstruction::TrajectoryOpKind kind : kinds)
	{
		const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(kind);
		const QString label = algo
			? QString::fromUtf8(algo->displayName(m_useChinese))
			: opKindLabel(kind, m_useChinese);
		auto* item = new QListWidgetItem(label, m_palette);
		item->setTextAlignment(Qt::AlignHCenter | Qt::AlignVCenter);
		item->setData(Qt::UserRole, static_cast<int>(kind));
	}
}

void TrajectoryEditPageWidget::runPreviewIfEnabled()
{
	if (!m_previewCheck || !m_previewCheck->isChecked() || !m_session || !m_pipeline)
	{
		return;
	}
	if (m_pipeline->ops().empty())
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

RobotInstruction::TrajectoryOpDescriptor TrajectoryEditPageWidget::makeDefaultOp(
	const RobotInstruction::TrajectoryOpKind kind) const
{
	const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(kind);
	if (algo)
	{
		return algo->makeDefaultDescriptor(defaultScopeForNewOp());
	}
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
	if (!m_session || !m_pipeline || m_flushingParams)
	{
		return;
	}
	m_flushingParams = true;
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
	m_flushingParams = false;
}

void TrajectoryEditPageWidget::refreshScopeFieldVisibility()
{
}

void TrajectoryEditPageWidget::refreshParamPanelForKind(const RobotInstruction::TrajectoryOpKind kind)
{
	(void)kind;
}

void TrajectoryEditPageWidget::loadSelectedOpToParams()
{
	if (m_pendingLoadSelectedOp)
	{
		return;
	}
	m_pendingLoadSelectedOp = true;
	QTimer::singleShot(0, this, [this]() {
		m_pendingLoadSelectedOp = false;
		loadSelectedOpToParamsImpl();
	});
}

void TrajectoryEditPageWidget::loadSelectedOpToParamsImpl()
{
	if (m_committingApply)
	{
		return;
	}
	if (!m_pipeline || !m_paramPanel || m_pipeline->selectedOpIndex() < 0)
	{
		return;
	}
	if (m_paramPanel->isRebuilding())
	{
		QTimer::singleShot(0, this, [this]() { loadSelectedOpToParamsImpl(); });
		return;
	}
	const RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->selectedOp();
	const trajectory_algo::ITrajectoryOp* algo =
		RobotInstruction::trajectoryOpGet(op.kind);
	m_loadingParams = true;
	m_paramPanel->setLoading(true);
	m_paramPanel->rebuildForOp(op, algo);
	m_loadingParams = false;
	m_paramPanel->setLoading(false);
	updateTransformActionButtons(op, m_previewCheck, m_applyBtn, m_readOnly);
}

void TrajectoryEditPageWidget::applyParamsToSelectedOp()
{
	if (!m_pipeline || m_pipeline->selectedOpIndex() < 0 || !m_paramPanel)
	{
		return;
	}
	RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->selectedOp();
	const std::string storedGroupId = op.scope.groupId;
	const trajectory_algo::ITrajectoryOp* algo =
		RobotInstruction::trajectoryOpGet(op.kind);
	std::string err;
	if (!m_paramPanel->applyTo(op, algo, &err))
	{
		return;
	}
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
	m_loadingParams = true;
	m_paramPanel->setLoading(true);
	m_pipeline->updateSelectedOp(op);
	m_paramPanel->setLoading(false);
	m_loadingParams = false;
	updateTransformActionButtons(op, m_previewCheck, m_applyBtn, m_readOnly);
	syncSessionParams();
}

void TrajectoryEditPageWidget::fillScopeFromUi(RobotInstruction::OpScope& scope) const
{
	(void)scope;
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

bool TrajectoryEditPageWidget::reconcilePipelineScopes()
{
	if (!m_store || !m_pipeline)
	{
		return false;
	}
	const RobotInstruction::RobotProgram* prog =
		m_store->activeCatalog().findProgram(m_store->activeProgramIdUtf8());
	if (!prog)
	{
		return false;
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
		m_pipeline->blockSignals(true);
		m_pipeline->setOps(ops);
		m_pipeline->blockSignals(false);
	}
	return changed;
}

void TrajectoryEditPageWidget::syncScopeComboFromSelectedOp()
{
	if (!m_pipeline || !m_scopeGroupCombo || m_pipeline->selectedOpIndex() < 0)
	{
		return;
	}
	const RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->selectedOp();
	m_scopeGroupCombo->blockSignals(true);
	if (op.scope.kind == RobotInstruction::OpScope::Kind::Group && !op.scope.groupId.empty())
	{
		const int idx = m_scopeGroupCombo->findData(QString::fromStdString(op.scope.groupId));
		if (idx >= 0)
		{
			m_scopeGroupCombo->setCurrentIndex(idx);
		}
	}
	m_scopeGroupCombo->blockSignals(false);
}

void TrajectoryEditPageWidget::syncUiAfterProgramRevision()
{
	if (m_committingApply || (m_session && m_session->isApplying()))
	{
		return;
	}
	m_loadingParams = true;
	if (m_paramPanel)
	{
		m_paramPanel->setLoading(true);
	}
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
	const bool pipelineScopeChanged = reconcilePipelineScopes();
	if (m_pipeline && m_pipeline->selectedOpIndex() >= 0)
	{
		// Apply/Undo 后多数情况流水线描述符未变，避免 clearRows 全量重建
		if (pipelineScopeChanged)
		{
			loadSelectedOpToParams();
		}
		else
		{
			const RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->selectedOp();
			m_pipeline->updateSelectedOp(op);
			syncScopeComboFromSelectedOp();
		}
	}
	if (m_paramPanel)
	{
		m_paramPanel->setLoading(false);
	}
	m_loadingParams = false;
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
			RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->selectedOp();
			if (op.scope.kind == RobotInstruction::OpScope::Kind::Group)
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
}

void TrajectoryEditPageWidget::onPipelineSelectionChanged(int index)
{
	(void)index;
	if (m_loadingParams || !m_pipeline || m_pipeline->selectedOpIndex() < 0)
	{
		return;
	}
	loadSelectedOpToParams();
}

void TrajectoryEditPageWidget::onPreviewToggled(const bool checked)
{
	if (!m_session)
	{
		return;
	}
	if (checked)
	{
		runPreviewIfEnabled();
	}
	else
	{
		m_session->reset();
	}
}

void TrajectoryEditPageWidget::showPipelineContextMenu(const QPoint& pos)
{
	if (!m_pipeline || m_readOnly)
	{
		return;
	}
	const int idx = m_pipeline->selectedOpIndex();
	const int count = m_pipeline->count();
	const bool zh = m_useChinese;
	QMenu menu(this);
	QAction* removeAct = menu.addAction(zh ? QStringLiteral("移除块") : QStringLiteral("Remove block"));
	QAction* upAct = menu.addAction(zh ? QStringLiteral("上移") : QStringLiteral("Move up"));
	QAction* downAct = menu.addAction(zh ? QStringLiteral("下移") : QStringLiteral("Move down"));
	removeAct->setEnabled(idx >= 0);
	upAct->setEnabled(idx > 0);
	downAct->setEnabled(idx >= 0 && idx < count - 1);
	QAction* picked = menu.exec(m_pipeline->mapToGlobal(pos));
	if (!picked)
	{
		return;
	}
	if (picked == removeAct)
	{
		onRemovePipelineOpClicked();
	}
	else if (picked == upAct)
	{
		onMovePipelineOpUpClicked();
	}
	else if (picked == downAct)
	{
		onMovePipelineOpDownClicked();
	}
}

void TrajectoryEditPageWidget::onApplyClicked()
{
	if (!m_session)
	{
		return;
	}
	m_committingApply = true;
	if (m_pipeline)
	{
		const QSignalBlocker pipelineBlocker(m_pipeline);
		m_pipeline->setCurrentRow(-1);
	}
	if (m_paramPanel)
	{
		m_paramPanel->clear();
	}
	reconcilePipelineScopes();
	flushPipelineToSession();
	QString err;
	if (!m_session->apply(&err))
	{
		m_committingApply = false;
		QMessageBox::warning(
			this,
			m_useChinese ? QStringLiteral("应用") : QStringLiteral("Apply"),
			err);
		return;
	}
	if (m_pipeline)
	{
		const QSignalBlocker pipelineBlocker(m_pipeline);
		m_pipeline->setOps({});
	}
	if (m_session)
	{
		m_session->clearPipelineAfterCommit();
	}
	if (m_previewCheck)
	{
		const QSignalBlocker blocker(m_previewCheck);
		m_previewCheck->setChecked(false);
	}
	if (m_applyBtn)
	{
		m_applyBtn->setEnabled(false);
	}
	if (m_commandPage)
	{
		m_commandPage->refreshInstructionList();
	}
	refreshUndoButtons();
	m_committingApply = false;
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
	}
}

void TrajectoryEditPageWidget::onMovePipelineOpUpClicked()
{
	if (m_pipeline)
	{
		m_pipeline->moveSelectedOp(-1);
	}
}

void TrajectoryEditPageWidget::onMovePipelineOpDownClicked()
{
	if (m_pipeline)
	{
		m_pipeline->moveSelectedOp(1);
	}
}

void TrajectoryEditPageWidget::onSaveTemplateClicked()
{
	if (!m_pipeline)
	{
		return;
	}
	const nlohmann::json pipelineJson = RobotInstruction::trajectoryPipelineToJson(m_pipeline->ops());
	QSettings settings(QStringLiteral("CloudSim"), QStringLiteral("TrajectoryPipeline"));
	settings.setValue(
		QStringLiteral("pipelineJson"),
		QString::fromStdString(pipelineJson.dump()));
}

void TrajectoryEditPageWidget::onLoadTemplateClicked()
{
	QSettings settings(QStringLiteral("CloudSim"), QStringLiteral("TrajectoryPipeline"));
	const QString jsonText = settings.value(QStringLiteral("pipelineJson")).toString();
	if (jsonText.isEmpty())
	{
		return;
	}
	nlohmann::json pipelineJson = nlohmann::json::parse(jsonText.toStdString(), nullptr, false);
	if (pipelineJson.is_discarded())
	{
		return;
	}
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops;
	std::string err;
	if (!RobotInstruction::trajectoryPipelineFromJson(pipelineJson, ops, &err))
	{
		return;
	}
	if (m_pipeline)
	{
		m_pipeline->setOps(ops);
		syncSessionPipeline();
		runPreviewIfEnabled();
	}
}
