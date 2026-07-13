#include "TrajectoryEditPageWidget.h"

#include "FeaturePickTransform.h"

#include "BackendDataManager.h"
#include "BrepBackendData.h"
#include "MeshBackendData.h"
#include "PointCloudBackendData.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "ProgramEditCommand.h"
#include "ProgramEditService.h"
#include "RecipeBlueprint.h"
#include "RobotOsgUiTypes.h"
#include "RobotSimulationController.h"
#include "RobotSimulationDockWidget.h"
#include "RawTrajectory.h"
#include "UnifiedTrajectory.h"
#include "SimulationCommandWidget.h"
#include "TrajectoryEditSession.h"
#include "TrajectoryEditObserver.h"
#include "TrajectoryGenerationPageWidget.h"
#include "TrajectoryOpParamPanel.h"
#include "TrajectoryPipelineListWidget.h"
#include "RobotProgramStore.h"
#include "RobotInstructionProgram.h"
#include "UiIconDecorators.h"

#include <ITrajectoryOp.h>
#include "TrajectoryOpBridge.h"

#include <json.hpp>

#include <QCoreApplication>
#include <QCheckBox>
#include <QComboBox>
#include <QFont>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QSettings>
#include <QSizePolicy>
#include <QUuid>
#include <QSignalBlocker>
#include <QTimer>
#include <QVBoxLayout>

#include <QDrag>
#include <QMimeData>

#include <algorithm>
#include <cstring>

namespace
{
constexpr int kTrajectoryBlockPaletteMinWidth = 158;
constexpr int kTrajectoryControlHeight = 26;

/// 调色板拖放须携带 kMimeType，否则流水线只会出现“幽灵项”（有显示无数据）
class TrajectoryOpPaletteWidget : public QListWidget
{
public:
	explicit TrajectoryOpPaletteWidget(QWidget* parent = nullptr)
		: QListWidget(parent)
	{
		setDragEnabled(true);
		setSpacing(2);
		setTextElideMode(Qt::ElideNone);
		setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
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
		return zh ? QStringLiteral("轴反向") : QStringLiteral("Axis Reverse");
	case RobotInstruction::TrajectoryOpKind::Delete:
		return zh ? QStringLiteral("删除") : QStringLiteral("Delete");
	case RobotInstruction::TrajectoryOpKind::Duplicate:
		return zh ? QStringLiteral("复制") : QStringLiteral("Duplicate");
	case RobotInstruction::TrajectoryOpKind::Reorder:
		return zh ? QStringLiteral("固定姿态") : QStringLiteral("Fixed Orientation");
	case RobotInstruction::TrajectoryOpKind::Resample:
		return zh ? QStringLiteral("重采样") : QStringLiteral("Resample");
	case RobotInstruction::TrajectoryOpKind::OffsetAlongNormal:
		return zh ? QStringLiteral("法向偏移") : QStringLiteral("Offset Along Normal");
	case RobotInstruction::TrajectoryOpKind::OffsetLateral:
		return zh ? QStringLiteral("横向偏移") : QStringLiteral("Offset Lateral");
	case RobotInstruction::TrajectoryOpKind::SmoothPose:
		return zh ? QStringLiteral("姿态平滑") : QStringLiteral("Smooth Pose");
	case RobotInstruction::TrajectoryOpKind::AssignBlend:
		return zh ? QStringLiteral("过渡半径") : QStringLiteral("Assign Blend");
	case RobotInstruction::TrajectoryOpKind::AssignSpeedZone:
		return zh ? QStringLiteral("速度区") : QStringLiteral("Assign Speed");
	case RobotInstruction::TrajectoryOpKind::Weave:
		return zh ? QStringLiteral("摆动") : QStringLiteral("Weave");
	case RobotInstruction::TrajectoryOpKind::ReachabilityFilter:
		return zh ? QStringLiteral("可达性过滤") : QStringLiteral("Reachability Filter");
	case RobotInstruction::TrajectoryOpKind::ExternalAxisSearch:
		return zh ? QStringLiteral("外部轴搜索") : QStringLiteral("External Axis Search");
	case RobotInstruction::TrajectoryOpKind::Approach:
		return zh ? QStringLiteral("进刀") : QStringLiteral("Approach");
	case RobotInstruction::TrajectoryOpKind::Retract:
		return zh ? QStringLiteral("退刀") : QStringLiteral("Retract");
	case RobotInstruction::TrajectoryOpKind::ProjectToGeometry:
		return zh ? QStringLiteral("轨迹投影") : QStringLiteral("ProjectToGeometry");
	case RobotInstruction::TrajectoryOpKind::Translate:
	default:
		return zh ? QStringLiteral("平移") : QStringLiteral("Translate");
	}
}

bool validatePipelineConstraints(
	const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops,
	const bool chinese,
	QString& outError)
{
	std::string err;
	if (!RobotInstruction::validateTrajectoryPipeline(ops, &err))
	{
		outError = err.empty()
			? (chinese ? QStringLiteral("流水线参数无效") : QStringLiteral("Invalid pipeline parameters"))
			: QString::fromStdString(err);
		return false;
	}
	return true;
}

void updateTransformActionButtons(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	QCheckBox* previewCheck,
	QPushButton* applyBtn,
	const bool readOnly)
{
	const trajectory_algo::ITrajectoryOp* algo = RobotInstruction::trajectoryOpGet(op.kind);
	const bool unifiedOnlyOp = op.kind == RobotInstruction::TrajectoryOpKind::Approach
		|| op.kind == RobotInstruction::TrajectoryOpKind::Retract
		|| op.kind == RobotInstruction::TrajectoryOpKind::Resample
		|| op.kind == RobotInstruction::TrajectoryOpKind::OffsetAlongNormal
		|| op.kind == RobotInstruction::TrajectoryOpKind::OffsetLateral
		|| op.kind == RobotInstruction::TrajectoryOpKind::SmoothPose
		|| op.kind == RobotInstruction::TrajectoryOpKind::AssignBlend
		|| op.kind == RobotInstruction::TrajectoryOpKind::AssignSpeedZone
		|| op.kind == RobotInstruction::TrajectoryOpKind::Weave
		|| op.kind == RobotInstruction::TrajectoryOpKind::ReachabilityFilter
		|| op.kind == RobotInstruction::TrajectoryOpKind::ExternalAxisSearch
		|| op.kind == RobotInstruction::TrajectoryOpKind::Delete
		|| op.kind == RobotInstruction::TrajectoryOpKind::Duplicate
		|| op.kind == RobotInstruction::TrajectoryOpKind::ProjectToGeometry;
	const bool canPosePreview = algo
		&& trajectory_algo::hasCapability(
			algo->capabilities(),
			trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform);
	const bool canPipelineOp = canPosePreview || unifiedOnlyOp;
	if (previewCheck)
	{
		previewCheck->setEnabled(canPipelineOp && !readOnly);
	}
	if (applyBtn)
	{
		applyBtn->setEnabled(canPipelineOp && !readOnly);
	}
}

} // namespace

TrajectoryEditPageWidget::TrajectoryEditPageWidget(QWidget* parent)
	: QWidget(parent)
{
	RobotInstruction::ensureTrajectoryOpBuiltinsRegistered();
	RobotInstruction::ensureTrajectoryOpConfigsLoaded(
		QCoreApplication::applicationDirPath().toStdString());
	m_observer = new TrajectoryEditObserver(this);

	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(4, 4, 4, 4);
	root->setSpacing(3);

	// 工艺模板 - 紧凑布局
	m_rawGroupBox = new QGroupBox(QStringLiteral("工艺模板"), this);
	auto* rawLayout = new QHBoxLayout(m_rawGroupBox);
	rawLayout->setContentsMargins(4, 2, 4, 2);
	rawLayout->setSpacing(3);
	m_rawStatusLabel = new QLabel(this);
	rawLayout->addWidget(m_rawStatusLabel, 1);
	m_rawRecipeCombo = new QComboBox(m_rawGroupBox);
	m_rawRecipeCombo->addItem(QStringLiteral("焊缝"), QStringLiteral("weld"));
	m_rawRecipeCombo->addItem(QStringLiteral("涂胶"), QStringLiteral("glue"));
	m_rawRecipeCombo->addItem(QStringLiteral("打磨"), QStringLiteral("grind"));
	rawLayout->addWidget(m_rawRecipeCombo);
	m_rawApplyBtn = new QPushButton(QStringLiteral("填充"), m_rawGroupBox);
	rawLayout->addWidget(m_rawApplyBtn);
	m_rawEmitBtn = new QPushButton(QStringLiteral("生成"), m_rawGroupBox);
	rawLayout->addWidget(m_rawEmitBtn);
	root->addWidget(m_rawGroupBox);

	// 作用域行 - 单行紧凑布局
	auto* scopeRow = new QHBoxLayout;
	scopeRow->setSpacing(3);
	m_programLabel = new QLabel(QStringLiteral("程序"), this);
	scopeRow->addWidget(m_programLabel);
	m_programCombo = new QComboBox(this);
	scopeRow->addWidget(m_programCombo, 1);
	m_groupLabel = new QLabel(QStringLiteral("组"), this);
	scopeRow->addWidget(m_groupLabel);
	m_groupCombo = new QComboBox(this);
	scopeRow->addWidget(m_groupCombo, 1);
	root->addLayout(scopeRow);

	// 调色板 + 流水线 - 主要区域，最大拉伸
	auto* bodyRow = new QHBoxLayout;
	bodyRow->setSpacing(3);
	const QFont blockFont = font();
	m_palette = new TrajectoryOpPaletteWidget(this);
	m_palette->setFont(blockFont);
	m_palette->setMinimumWidth(kTrajectoryBlockPaletteMinWidth);
	m_palette->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
	bodyRow->addWidget(m_palette, 1);

	m_pipeline = new TrajectoryPipelineListWidget(this);
	m_pipeline->setFont(blockFont);
	m_pipeline->setTextElideMode(Qt::ElideNone);
	m_pipeline->setHorizontalScrollBarPolicy(Qt::ScrollBarAsNeeded);
	m_pipeline->setDefaultOpFactory([this](const RobotInstruction::TrajectoryOpKind kind) {
		return makeDefaultOp(kind);
	});
	bodyRow->addWidget(m_pipeline, 2);
	root->addLayout(bodyRow, 3);

	// 参数区域 - 可折叠，有限高度
	m_paramGroupBox = new QGroupBox(QStringLiteral("参数"), this);
	m_paramGroupBox->setCheckable(true);
	m_paramGroupBox->setChecked(true);
	m_paramGroupBox->setFont(blockFont);
	// 字号继承算法块，仅覆盖控件尺寸
	m_paramGroupBox->setStyleSheet(QStringLiteral(
		"QGroupBox { font-weight: 500; }"
		"QGroupBox QComboBox { min-height: %1px; max-height: %1px; padding: 2px 6px; }"
		"QGroupBox QSpinBox, QGroupBox QDoubleSpinBox { min-height: %1px; max-height: %1px; padding: 2px 6px; }"
		"QGroupBox QPushButton { padding: 2px 8px; min-height: %1px; max-height: %1px; }"
		"QGroupBox QCheckBox { spacing: 4px; }"
	).arg(kTrajectoryControlHeight));
	auto* paramBoxLayout = new QVBoxLayout(m_paramGroupBox);
	paramBoxLayout->setContentsMargins(4, 2, 4, 2);
	paramBoxLayout->setSpacing(0);
	m_scopeGroupCombo = new QComboBox(m_paramGroupBox);
	m_scopeGroupCombo->setFixedHeight(kTrajectoryControlHeight);
	m_scopeGroupCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_geometryBackendCombo = new QComboBox(m_paramGroupBox);
	m_geometryBackendCombo->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Fixed);
	m_geometryBackendPickBtn = new QPushButton(m_paramGroupBox);
	m_paramPanel = new TrajectoryOpParamPanel(m_paramGroupBox);
	m_paramPanel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
	m_paramPanel->setScopeGroupCombo(m_scopeGroupCombo);
	m_paramPanel->setGeometryBackendCombo(m_geometryBackendCombo);
	m_paramPanel->setGeometryBackendPickButton(m_geometryBackendPickBtn);
	paramBoxLayout->addWidget(m_paramPanel, 1);
	root->addWidget(m_paramGroupBox, 1);

	m_pipeline->setContextMenuPolicy(Qt::CustomContextMenu);

	// 操作按钮行 - 水平拉伸填满
	auto* actionRow1 = new QHBoxLayout;
	actionRow1->setSpacing(3);
	actionRow1->setContentsMargins(0, 0, 0, 0);
	m_previewCheck = new QCheckBox(QStringLiteral("预览"), this);
	m_previewCheck->setChecked(true);
	m_applyBtn = new QPushButton(QStringLiteral("应用"), this);
	m_resetBtn = new QPushButton(QStringLiteral("重置"), this);
	m_undoBtn = new QPushButton(QStringLiteral("撤销"), this);
	m_redoBtn = new QPushButton(QStringLiteral("重做"), this);
	m_saveTemplateBtn = new QPushButton(QStringLiteral("保存"), this);
	m_loadTemplateBtn = new QPushButton(QStringLiteral("加载"), this);
	actionRow1->addWidget(m_previewCheck);
	actionRow1->addWidget(m_applyBtn, 1);
	actionRow1->addWidget(m_resetBtn, 1);
	actionRow1->addWidget(m_undoBtn, 1);
	actionRow1->addWidget(m_redoBtn, 1);
	auto* actionRow2 = new QHBoxLayout;
	actionRow2->setSpacing(3);
	actionRow2->setContentsMargins(0, 0, 0, 0);
	actionRow2->addWidget(m_saveTemplateBtn, 1);
	actionRow2->addWidget(m_loadTemplateBtn, 1);
	root->addLayout(actionRow1);
	root->addLayout(actionRow2);

	rebuildPalette();

	connect(m_programCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &TrajectoryEditPageWidget::onProgramChanged);
	connect(m_groupCombo, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &TrajectoryEditPageWidget::onGroupChanged);
	connect(m_palette, &QListWidget::itemDoubleClicked, this, &TrajectoryEditPageWidget::onPaletteDoubleClicked);
	connect(m_pipeline, &TrajectoryPipelineListWidget::opsChanged, this, [this]() {
		setPipelineAppliedState(false, false);
		syncSessionPipeline();
		if (m_loadingParams || (m_paramPanel && m_paramPanel->isRebuilding()))
		{
			return;
		}
		schedulePreviewRun(160, false);
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

	UiIconDecorators::apply(m_applyBtn, UiIconId::Apply);
	UiIconDecorators::apply(m_resetBtn, UiIconId::Reset);
	UiIconDecorators::apply(m_undoBtn, UiIconId::Undo);
	UiIconDecorators::apply(m_redoBtn, UiIconId::Redo);
	UiIconDecorators::apply(m_saveTemplateBtn, UiIconId::SaveTemplate);
	UiIconDecorators::apply(m_loadTemplateBtn, UiIconId::LoadTemplate);
	UiIconDecorators::apply(m_rawApplyBtn, UiIconId::FillRecipe);
	UiIconDecorators::apply(m_rawEmitBtn, UiIconId::EmitProgram);

	connect(m_paramPanel, &TrajectoryOpParamPanel::paramsChanged, this, [this]() {
		if (!m_loadingParams)
		{
			setPipelineAppliedState(false, false);
			applyParamsToSelectedOp();
			if (!m_flushingParams && m_previewCheck && m_previewCheck->isChecked()
				&& m_session && !m_session->isPreviewActive())
			{
				schedulePreviewRun(100, false);
			}
		}
	});
	connect(m_scopeGroupCombo, QOverload<int>::of(&QComboBox::activated), this, [this]() {
		if (!m_loadingParams)
		{
			setPipelineAppliedState(false, false);
			applyParamsToSelectedOp();
			if (!m_flushingParams && m_previewCheck && m_previewCheck->isChecked()
				&& m_session && !m_session->isPreviewActive())
			{
				schedulePreviewRun(100, false);
			}
		}
	});
	const auto onGeometryBackendComboChanged = [this]() {
		if (!m_loadingParams)
		{
			setPipelineAppliedState(false, false);
			applyParamsToSelectedOp();
			if (!m_flushingParams && m_previewCheck && m_previewCheck->isChecked()
				&& m_session && !m_session->isPreviewActive())
			{
				schedulePreviewRun(100, false);
			}
		}
	};
	connect(
		m_geometryBackendCombo,
		QOverload<int>::of(&QComboBox::currentIndexChanged),
		this,
		onGeometryBackendComboChanged);
	connect(
		m_geometryBackendCombo,
		QOverload<int>::of(&QComboBox::activated),
		this,
		onGeometryBackendComboChanged);
	connect(m_geometryBackendPickBtn, &QPushButton::clicked, this, [this]() {
		if (!m_host || !m_geometryBackendCombo)
		{
			return;
		}
		const QString selectedId = m_host->selectedBackendId();
		if (selectedId.isEmpty())
		{
			return;
		}
		const int idx = m_geometryBackendCombo->findData(selectedId);
		if (idx < 0)
		{
			return;
		}
		m_geometryBackendCombo->setCurrentIndex(idx);
		if (!m_loadingParams)
		{
			setPipelineAppliedState(false, false);
			applyParamsToSelectedOp();
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
		m_groupLabel->setText(zh ? QStringLiteral("组") : QStringLiteral("Grp"));
	}
	if (m_paramGroupBox)
	{
		m_paramGroupBox->setTitle(zh ? QStringLiteral("参数") : QStringLiteral("Parameters"));
	}
	if (m_paramPanel)
	{
		m_paramPanel->setUseChinese(zh);
	}
	if (m_geometryBackendPickBtn)
	{
		m_geometryBackendPickBtn->setText(
			zh ? QStringLiteral("选中填充") : QStringLiteral("Fill"));
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
		m_saveTemplateBtn->setText(zh ? QStringLiteral("保存") : QStringLiteral("Save"));
	}
	if (m_loadTemplateBtn)
	{
		m_loadTemplateBtn->setText(zh ? QStringLiteral("加载") : QStringLiteral("Load"));
	}
	if (m_rawGroupBox)
	{
		m_rawGroupBox->setTitle(zh ? QStringLiteral("工艺模板") : QStringLiteral("Recipe"));
	}
	if (m_rawApplyBtn)
	{
		m_rawApplyBtn->setText(zh ? QStringLiteral("填充") : QStringLiteral("Fill"));
	}
	if (m_rawEmitBtn)
	{
		m_rawEmitBtn->setText(zh ? QStringLiteral("生成") : QStringLiteral("Emit"));
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
	if (m_observer)
	{
		m_observer->bindEditService(m_editService);
	}
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
	if (m_observer)
	{
		m_observer->bindSession(m_session);
		m_observer->bindEditService(m_editService);
	}
	if (m_session && m_store)
	{
		syncBoundPathPlanFromSession();
	}
	if (m_session)
	{
		connect(m_session, &TrajectoryEditSession::rawTrajectoryChanged, this, [this]() {
			setPipelineAppliedState(false, false);
			refreshRawTrajectoryStatus();
		});
		connect(m_session, &TrajectoryEditSession::pathPlanBound, this, [this](const std::string&) {
			if (m_pipeline)
			{
				m_loadingParams = true;
				m_pipeline->blockSignals(true);
				m_pipeline->setOps({});
				m_pipeline->blockSignals(false);
				m_loadingParams = false;
			}
			refreshProgramAndGroupCombos();
			setPipelineAppliedState(false, false);
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

void TrajectoryEditPageWidget::setPipelineAppliedState(const bool applied, const bool announce)
{
	const bool changed = (m_pipelineAppliedSinceLastRawChange != applied);
	m_pipelineAppliedSinceLastRawChange = applied;
	refreshRawTrajectoryStatus();
	if (!announce || !changed || !m_host)
	{
		return;
	}
	if (applied)
	{
		m_host->appendRunInfo(m_useChinese
				? QStringLiteral("结果已落盘，生成程序入口已禁用")
				: QStringLiteral("Result committed, emit program disabled"));
	}
	else
	{
		m_host->appendRunInfo(m_useChinese
				? QStringLiteral("检测到新编辑，可重新生成程序")
				: QStringLiteral("New edits detected, emit program re-enabled"));
	}
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
	QString status = zh ? QStringLiteral("原始轨迹：%1 点").arg(n)
						: QStringLiteral("Raw trajectory: %1 points").arg(n);
	if (m_pipelineAppliedSinceLastRawChange)
	{
		status += zh ? QStringLiteral("（已应用，生成已禁用）")
					 : QStringLiteral(" (Applied, emit disabled)");
	}
	m_rawStatusLabel->setText(status);
	if (m_rawApplyBtn)
	{
		m_rawApplyBtn->setEnabled(!m_readOnly);
	}
	if (m_rawEmitBtn)
	{
		m_rawEmitBtn->setEnabled(!m_readOnly && m_store != nullptr && !m_pipelineAppliedSinceLastRawChange);
	}
}

std::string TrajectoryEditPageWidget::resolvePreviewBackendId(const RobotInstruction::RawTrajectory& traj) const
{
	const std::string backendId = RobotInstruction::rawTrajectoryWorkpieceBackendId(traj);
	if (!backendId.empty())
	{
		return backendId;
	}
	return {};
}

void TrajectoryEditPageWidget::showRawTrajectoryPreview(
	const RobotInstruction::RawTrajectory& traj,
	const bool posesAlreadyWorldMm)
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
	if (traj.points.empty())
	{
		return;
	}
	RobotOsgUi::RawTrajectoryPreviewOptions options;
	options.showAxisX = true;
	options.showAxisY = true;
	options.showAxisZ = true;
	options.showAxes = true;
	options.axisInterval = 0;
	std::string err;
	if (posesAlreadyWorldMm)
	{
		feature_pick_transform::applyWorldRawTrajectoryPreviewToOsg(osg, traj, options, &err);
	}
	else
	{
		const std::string backendId = resolvePreviewBackendId(traj);
		if (backendId.empty())
		{
			if (m_host)
			{
				m_host->appendRunWarning(
					m_useChinese ? QStringLiteral("轨迹预览：FeatureSpec 缺少 workpiece.backendIdUtf8")
								 : QStringLiteral("Trajectory preview: missing workpiece.backendIdUtf8"));
			}
			return;
		}
		feature_pick_transform::applyRawTrajectoryPreviewToOsg(osg, backendId, traj, options, &err);
	}
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
	setPipelineAppliedState(false, false);
	const QString recipe = m_rawRecipeCombo->currentData().toString();
	RobotInstruction::RecipeKind recipeKind = RobotInstruction::RecipeKind::Weld;
	if (recipe == QStringLiteral("glue"))
	{
		recipeKind = RobotInstruction::RecipeKind::Glue;
	}
	else if (recipe == QStringLiteral("grind"))
	{
		recipeKind = RobotInstruction::RecipeKind::Grind;
	}
	applyRecipePresetByKind(recipeKind);
}

void TrajectoryEditPageWidget::applyRecipePresetByKind(const RobotInstruction::RecipeKind recipeKind)
{
	const std::vector<RobotInstruction::TrajectoryOpDescriptor> ops =
		RobotInstruction::buildRecipePreset(recipeKind);
	if (m_pipeline)
	{
		m_pipeline->setOps(ops);
		syncSessionPipeline();
		schedulePreviewRun(120, false);
	}
	if (m_host)
	{
		m_host->appendRunInfo(m_useChinese ? QStringLiteral("工艺模板已填充到流水线")
			: QStringLiteral("Recipe preset inserted to pipeline"));
	}
}

void TrajectoryEditPageWidget::onRawEmitProgram()
{
	if (!m_session || !m_store || !m_session->hasRawTrajectory())
	{
		return;
	}
	if (m_pipelineAppliedSinceLastRawChange)
	{
		QMessageBox::information(
			this,
			m_useChinese ? QStringLiteral("生成") : QStringLiteral("Emit"),
			m_useChinese ? QStringLiteral("已应用后请勿再生成程序，避免覆盖应用结果")
						 : QStringLiteral("Emit is disabled after Apply to avoid overriding applied result"));
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
	const std::string* pathPlanIdPtr = nullptr;
	std::string boundPathPlanId;
	if (m_session && !m_session->boundPathPlanId().empty())
	{
		boundPathPlanId = m_session->boundPathPlanId();
		pathPlanIdPtr = &boundPathPlanId;
	}
	if (!RobotInstruction::emitRawTrajectoryToProgram(
			worldTraj, *prog, &err, &emittedGroupId, pathPlanIdPtr))
	{
		QMessageBox::warning(this, QStringLiteral("生成"), QString::fromStdString(err));
		return;
	}
	if (m_commandPage)
	{
		m_commandPage->refreshInstructionList();
	}
	if (!emittedGroupId.empty())
	{
		m_selectedGroupId = emittedGroupId;
	}
	if (pathPlanIdPtr && m_store)
	{
		if (RobotInstruction::PathPlanInstruction* pp = m_store->activeCatalog().findPathPlan(
				m_store->activeCatalog().activeProgramId(),
				*pathPlanIdPtr))
		{
			pp->setOutputGroupId(emittedGroupId);
		}
	}
	refreshProgramAndGroupCombos();
	if (m_simController && prog)
	{
		std::vector<std::shared_ptr<RobotInstruction::Base>> flat;
		RobotInstruction::flattenInstructionsRecursive(prog->steps, flat);
		for (const std::shared_ptr<RobotInstruction::Base>& ins : flat)
		{
			if (ins && RobotInstruction::isMotionWaypointType(ins->type()))
			{
				m_simController->syncInstructionRenderMatricesFromWorldPose(ins);
			}
		}
		m_simController->setRawTrajectoryPreviewActive(false);
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
		const std::string featureId = RobotInstruction::rawTrajectoryFeatureId(*src);
		const QString groupName = featureId.empty()
			? QStringLiteral("RawTrajectory")
			: QString::fromStdString(featureId);
		m_host->appendRunInfo(m_useChinese
			? QStringLiteral("已写入主程序，分组「%1」").arg(groupName)
			: QStringLiteral("Written to main program, group \"%1\"").arg(groupName));
	}
	resetTrajectoryGenerationPages();
}

void TrajectoryEditPageWidget::resetTrajectoryGenerationPages()
{
	if (!m_simController || !m_simController->simulationDock())
	{
		return;
	}
	if (TrajectoryGenerationPageWidget* gen = m_simController->simulationDock()->trajectoryGenerationPage())
	{
		gen->resetAfterTrajectoryCommit();
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

void TrajectoryEditPageWidget::syncBoundPathPlanFromSession()
{
	if (!m_session || !m_store || !m_pipeline)
	{
		return;
	}
	const std::string pathPlanId = m_session->boundPathPlanId();
	if (pathPlanId.empty())
	{
		return;
	}
	if (RobotInstruction::PathPlanInstruction* pp = m_store->activeCatalog().findPathPlan(
			m_store->activeCatalog().activeProgramId(),
			pathPlanId))
	{
		m_loadingParams = true;
		m_pipeline->blockSignals(true);
		m_pipeline->setOps(pp->pipeline());
		m_pipeline->blockSignals(false);
		m_loadingParams = false;
	}
}

void TrajectoryEditPageWidget::restoreBoundPathPlanForEdit()
{
	if (!m_session)
	{
		return;
	}
	(void)m_session->reloadBoundPathPlanFromStore();
	syncBoundPathPlanFromSession();
	refreshProgramAndGroupCombos();

	bool applied = false;
	if (m_store && !m_session->boundPathPlanId().empty())
	{
		if (const RobotInstruction::PathPlanInstruction* pp = m_store->activeCatalog().findPathPlan(
				m_store->activeCatalog().activeProgramId(),
				m_session->boundPathPlanId()))
		{
			applied = pp->phase() == RobotInstruction::PathPlanPhase::Applied;
		}
	}
	setPipelineAppliedState(applied, false);
	refreshRawTrajectoryStatus();

	if (m_session && m_pipeline)
	{
		m_session->syncPipelineEngine(m_pipeline->ops());
	}
	if (m_previewCheck && m_previewCheck->isChecked() && m_session && m_session->hasRawTrajectory())
	{
		schedulePreviewRun(120, false);
	}
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
	if (!m_selectedGroupId.empty())
	{
		prevTopGroupId = QString::fromStdString(m_selectedGroupId);
	}
	else if (m_groupCombo && m_groupCombo->currentIndex() > 0)
	{
		prevTopGroupId = m_groupCombo->currentData().toString();
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
	else if (m_session && !m_session->boundPathPlanId().empty() && prog)
	{
		for (const RobotInstruction::InstructionGroup& group : prog->groups)
		{
			if (group.role == RobotInstruction::InstructionGroupRole::PathPlanOutput
				&& group.pathPlanInstructionId == m_session->boundPathPlanId()
				&& !group.memberInstructionIds.empty())
			{
				const int gIdx = m_groupCombo->findData(QString::fromStdString(group.id));
				if (gIdx >= 0)
				{
					topGroupIdx = gIdx;
				}
				break;
			}
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
	refreshGeometryBackendCombo();
}

void TrajectoryEditPageWidget::refreshGeometryBackendCombo()
{
	if (!m_geometryBackendCombo)
	{
		return;
	}
	QString prevBackendId;
	if (m_pipeline && m_pipeline->selectedOpIndex() >= 0)
	{
		prevBackendId = QString::fromStdString(
			RobotInstruction::trajectoryOpProjectTargetBackendId(m_pipeline->selectedOp()));
	}
	if (prevBackendId.isEmpty() && m_geometryBackendCombo->currentIndex() >= 0)
	{
		prevBackendId = m_geometryBackendCombo->currentData().toString();
	}
	m_geometryBackendCombo->blockSignals(true);
	m_geometryBackendCombo->clear();
	if (m_host && m_host->document())
	{
		BackendDataManager& mgr = m_host->document()->backend();
		for (const std::shared_ptr<BackendDataBase>& data : mgr.listData())
		{
			if (!data || !data->hasGeometry())
			{
				continue;
			}
			const bool isPointCloud = static_cast<bool>(std::dynamic_pointer_cast<PointCloudBackendData>(data));
			const bool isMesh = static_cast<bool>(std::dynamic_pointer_cast<MeshBackendData>(data));
			const bool isBrep = static_cast<bool>(std::dynamic_pointer_cast<BrepBackendData>(data));
			if (!isPointCloud && !isMesh && !isBrep)
			{
				continue;
			}
			const QString backendId = QString::fromStdString(data->id());
			if (backendId.startsWith(QStringLiteral("RobotURDF_")))
			{
				continue;
			}
			const QString label = QString::fromStdString(data->name()).isEmpty()
				? backendId
				: QStringLiteral("%1 (%2)").arg(
					QString::fromStdString(data->name()),
					backendId);
			m_geometryBackendCombo->addItem(label, backendId);
		}
	}
	if (!prevBackendId.isEmpty())
	{
		const int idx = m_geometryBackendCombo->findData(prevBackendId);
		if (idx >= 0)
		{
			m_geometryBackendCombo->setCurrentIndex(idx);
		}
	}
	m_geometryBackendCombo->blockSignals(false);
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

void TrajectoryEditPageWidget::runPreviewIfEnabled(const bool showWarnings)
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
	if (m_session->hasRawTrajectory())
	{
		RobotInstruction::RawTrajectory previewRaw{};
		QString previewErr;
		if (!m_session->buildRawPreviewWithPipeline(m_pipeline->ops(), previewRaw, &previewErr))
		{
			if (showWarnings && !previewErr.isEmpty())
			{
				QMessageBox::warning(
					this,
					m_useChinese ? QStringLiteral("预览") : QStringLiteral("Preview"),
					previewErr);
			}
			return;
		}
		if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
		{
			osg->clearInstructionPoseAxes();
		}
		showRawTrajectoryPreview(previewRaw, true);
		return;
	}
	QString err;
	if (!m_session->previewPipeline(m_pipeline->ops(), &err))
	{
		if (showWarnings && !err.isEmpty())
		{
			QMessageBox::warning(
				this,
				m_useChinese ? QStringLiteral("预览") : QStringLiteral("Preview"),
				err);
		}
	}
}

void TrajectoryEditPageWidget::schedulePreviewRun(const int delayMs, const bool showWarnings)
{
	++m_previewScheduleToken;
	const int token = m_previewScheduleToken;
	QTimer::singleShot(std::max(0, delayMs), this, [this, token, showWarnings]() {
		if (token != m_previewScheduleToken)
		{
			return;
		}
		if (m_loadingParams || (m_paramPanel && m_paramPanel->isRebuilding()))
		{
			return;
		}
		runPreviewIfEnabled(showWarnings);
	});
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
	auto op = RobotInstruction::trajectoryOpDefaultUnified(kind, defaultScopeForNewOp());
	op.opId = QUuid::createUuid().toString(QUuid::WithoutBraces).toStdString();
	return op;
}

void TrajectoryEditPageWidget::syncSessionPipeline()
{
	if (!m_session || !m_pipeline)
	{
		return;
	}
	if (!m_session->boundPathPlanId().empty())
	{
		m_session->updatePipelineOps(m_pipeline->ops(), false);
	}
	else
	{
		m_session->setPipeline(m_pipeline->ops());
	}
	m_session->syncPipelineEngine(m_pipeline->ops());
}

void TrajectoryEditPageWidget::syncSessionParams(const bool skipPreviewReapply)
{
	if (!m_session || !m_pipeline)
	{
		return;
	}
	m_session->updatePipelineOps(m_pipeline->ops(), false);
	m_session->syncPipelineEngine(m_pipeline->ops());
	if (!skipPreviewReapply && m_previewCheck && m_previewCheck->isChecked() && !m_loadingParams
		&& !(m_paramPanel && m_paramPanel->isRebuilding()))
	{
		if (m_session->hasRawTrajectory())
		{
			const int nodeIndex = m_pipeline->selectedOpIndex();
			if (nodeIndex >= 0)
			{
				m_session->runPipelineEngineFrom(static_cast<std::size_t>(nodeIndex), nullptr);
			}
		}
		schedulePreviewRun(80, false);
	}
}

void TrajectoryEditPageWidget::flushPipelineToSession(const bool forApply)
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
	syncProjectBackendFromComboToPipeline(forApply);
	if (m_pipeline->selectedOpIndex() >= 0)
	{
		applyParamsToSelectedOp(forApply);
	}
	else
	{
		syncSessionParams(forApply);
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

void TrajectoryEditPageWidget::syncProjectBackendFromComboToPipeline(const bool allProjectOps)
{
	if (!m_pipeline || !m_geometryBackendCombo || m_geometryBackendCombo->currentIndex() < 0)
	{
		return;
	}
	const std::string backendId =
		m_geometryBackendCombo->currentData().toString().toStdString();
	if (backendId.empty())
	{
		return;
	}
	const int sel = m_pipeline->selectedOpIndex();
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops = m_pipeline->ops();
	bool changed = false;
	for (std::size_t i = 0; i < ops.size(); ++i)
	{
		if (ops[i].kind != RobotInstruction::TrajectoryOpKind::ProjectToGeometry)
		{
			continue;
		}
		if (!allProjectOps && sel >= 0 && static_cast<int>(i) != sel)
		{
			continue;
		}
		const std::string currentBackendId =
			RobotInstruction::trajectoryOpProjectTargetBackendId(ops[i]);
		if (currentBackendId == backendId)
		{
			continue;
		}
		RobotInstruction::trajectoryOpSetProjectTargetBackendId(ops[i], backendId);
		changed = true;
	}
	if (!changed)
	{
		return;
	}
	m_pipeline->blockSignals(true);
	m_pipeline->setOps(ops);
	if (sel >= 0 && sel < m_pipeline->count())
	{
		m_pipeline->setCurrentRow(sel);
	}
	m_pipeline->blockSignals(false);
}

std::vector<RobotInstruction::TrajectoryOpDescriptor> TrajectoryEditPageWidget::buildPipelineOpsForApply() const
{
	if (!m_pipeline)
	{
		return {};
	}
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops = m_pipeline->ops();
	if (!m_geometryBackendCombo || m_geometryBackendCombo->currentIndex() < 0)
	{
		return ops;
	}
	const std::string backendId =
		m_geometryBackendCombo->currentData().toString().toStdString();
	if (backendId.empty())
	{
		return ops;
	}
	for (RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		if (op.kind == RobotInstruction::TrajectoryOpKind::ProjectToGeometry)
		{
			RobotInstruction::trajectoryOpSetProjectTargetBackendId(op, backendId);
		}
	}
	return ops;
}

void TrajectoryEditPageWidget::applyParamsToSelectedOp(const bool skipPreviewReapply)
{
	if (!m_pipeline || m_pipeline->selectedOpIndex() < 0 || !m_paramPanel)
	{
		return;
	}
	const int opIndex = m_pipeline->selectedOpIndex();
	RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->opAt(opIndex);
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
	syncProjectBackendFromComboToPipeline(false);
	m_loadingParams = true;
	m_paramPanel->setLoading(true);
	m_pipeline->updateOpAt(opIndex, op);
	m_paramPanel->setLoading(false);
	m_loadingParams = false;
	updateTransformActionButtons(op, m_previewCheck, m_applyBtn, m_readOnly);
	syncSessionParams(skipPreviewReapply);
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
	std::string preferredOutputGroupId;
	if (m_session && !m_session->boundPathPlanId().empty())
	{
		for (const RobotInstruction::InstructionGroup& group : prog->groups)
		{
			if (group.role == RobotInstruction::InstructionGroupRole::PathPlanOutput
				&& group.pathPlanInstructionId == m_session->boundPathPlanId()
				&& !group.memberInstructionIds.empty())
			{
				preferredOutputGroupId = group.id;
				break;
			}
		}
	}
	std::vector<RobotInstruction::TrajectoryOpDescriptor> ops = m_pipeline->ops();
	bool changed = false;
	for (RobotInstruction::TrajectoryOpDescriptor& op : ops)
	{
		// 防御性修正：Group + 空 groupId 是无效状态，回退到 EntireProgram
		if (op.scope.kind == RobotInstruction::OpScope::Kind::Group && op.scope.groupId.empty())
		{
			op.scope.kind = RobotInstruction::OpScope::Kind::EntireProgram;
			changed = true;
			continue;
		}
		if (op.scope.kind != RobotInstruction::OpScope::Kind::Group)
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
		if (!preferredOutputGroupId.empty())
		{
			fallbackId = preferredOutputGroupId;
		}
		if (fallbackId.empty() && !m_selectedGroupId.empty())
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

void TrajectoryEditPageWidget::syncGeometryBackendComboFromSelectedOp()
{
	if (!m_pipeline || !m_geometryBackendCombo || m_pipeline->selectedOpIndex() < 0)
	{
		return;
	}
	const RobotInstruction::TrajectoryOpDescriptor op = m_pipeline->selectedOp();
	const std::string targetBackendId =
		RobotInstruction::trajectoryOpProjectTargetBackendId(op);
	m_geometryBackendCombo->blockSignals(true);
	if (!targetBackendId.empty())
	{
		const int idx = m_geometryBackendCombo->findData(
			QString::fromStdString(targetBackendId));
		if (idx >= 0)
		{
			m_geometryBackendCombo->setCurrentIndex(idx);
		}
	}
	m_geometryBackendCombo->blockSignals(false);
}

void TrajectoryEditPageWidget::syncUiAfterProgramRevision()
{
	if (m_committingApply || (m_session && m_session->isApplying()))
	{
		return;
	}
	refreshUndoButtons();
	if (m_session && m_session->defersProgramRevisionUiSync())
	{
		refreshProgramAndGroupCombos();
		return;
	}
	m_loadingParams = true;
	if (m_paramPanel)
	{
		m_paramPanel->setLoading(true);
	}
	refreshProgramAndGroupCombos();
	if (m_session)
	{
		m_session->abandonPreview();
		if (m_store)
		{
			m_session->setContextProgramId(m_store->activeProgramIdUtf8());
			syncBoundPathPlanFromSession();
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
			syncGeometryBackendComboFromSelectedOp();
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
		schedulePreviewRun(0, true);
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
	syncProjectBackendFromComboToPipeline(true);
	if (m_pipeline && m_pipeline->selectedOpIndex() >= 0)
	{
		applyParamsToSelectedOp(true);
	}
	m_committingApply = true;
	reconcilePipelineScopes();
	std::vector<RobotInstruction::TrajectoryOpDescriptor> applyOps = buildPipelineOpsForApply();
	if (!applyOps.empty())
	{
		QString constraintErr;
		if (!validatePipelineConstraints(applyOps, m_useChinese, constraintErr))
		{
			m_committingApply = false;
			QMessageBox::warning(
				this,
				m_useChinese ? QStringLiteral("应用") : QStringLiteral("Apply"),
				constraintErr);
			return;
		}
	}
	if (m_pipeline)
	{
		const QSignalBlocker pipelineBlocker(m_pipeline);
		m_pipeline->setOps(applyOps);
		m_pipeline->setCurrentRow(-1);
	}
	if (m_paramPanel)
	{
		m_paramPanel->clear();
	}
	if (m_session)
	{
		m_session->updatePipelineOps(applyOps, true);
	}
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
	if (m_simController)
	{
		m_simController->setRawTrajectoryPreviewActive(false);
	}
	if (IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr)
	{
		osg->clearRawTrajectoryOverlay();
		osg->clearRawTrajectoryOverlayFrames();
	}
	if (m_simController)
	{
		m_simController->refreshInstructionPoseAxes(false);
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
	setPipelineAppliedState(true, true);
	m_committingApply = false;
	if (m_store)
	{
		const RobotInstruction::RobotProgram* prog =
			m_store->activeCatalog().findProgram(m_store->activeProgramIdUtf8());
		if (prog && !prog->groups.empty())
		{
			bool selectedStillValid = false;
			for (const RobotInstruction::InstructionGroup& group : prog->groups)
			{
				if (group.id == m_selectedGroupId)
				{
					selectedStillValid = true;
					break;
				}
			}
			if (!selectedStillValid)
			{
				m_selectedGroupId = prog->groups.front().id;
			}
		}
	}
	refreshProgramAndGroupCombos();
	resetTrajectoryGenerationPages();
}

void TrajectoryEditPageWidget::onResetClicked()
{
	setPipelineAppliedState(false, false);
	if (m_session)
	{
		m_session->reset();
		m_session->clearTrajectoryGeometryHistory();
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
		schedulePreviewRun(120, false);
	}
}
