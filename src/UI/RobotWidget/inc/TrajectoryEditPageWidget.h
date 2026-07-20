#ifndef ROBOTWIDGET_TRAJECTORYEDITPAGEWIDGET_H
#define ROBOTWIDGET_TRAJECTORYEDITPAGEWIDGET_H

/// @file TrajectoryEditPageWidget.h
/// @brief 轨迹编辑 Dock 子页：流水线 Preview/Apply

#include "robotwidget_global.h"

#include "TrajectoryPipelineTypes.h"

#include <QPoint>
#include <QWidget>
#include <string>

namespace RobotInstruction
{
struct RawTrajectory;
enum class RecipeKind;
} // namespace RobotInstruction

class QCheckBox;

class QComboBox;

class QGroupBox;

class QLabel;

class QPushButton;

class QListWidget;

class QListWidgetItem;

class TrajectoryPipelineListWidget;

class TrajectoryEditSession;
class TrajectoryEditObserver;

class ProgramEditService;

class RobotProgramStore;

class SimulationCommandWidget;

class TrajectoryOpParamPanel;

class IRobotMainWindowHost;

class RobotSimulationController;

/// 轨迹编辑 Dock 子页：流水线 Preview/Apply

class ROBOTWIDGET_EXPORT TrajectoryEditPageWidget : public QWidget

{
	Q_OBJECT

public:
	explicit TrajectoryEditPageWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);

	void setReadOnly(bool readOnly);

	void bindStore(RobotProgramStore* store);

	void bindEditService(ProgramEditService* service);

	void bindSession(TrajectoryEditSession* session);

	void bindSimulationController(RobotSimulationController* controller);

	void bindCommandPage(SimulationCommandWidget* commandPage);

	void bindHost(IRobotMainWindowHost* host);

	void refreshProgramAndGroupCombos();
	void refreshGeometryBackendCombo();

	void syncBoundPathPlanFromSession();

	/// 「开始修改」：从 PathPlan 恢复算子流程并同步 UI 状态
	void restoreBoundPathPlanForEdit();

	void applyRecipePresetByKind(RobotInstruction::RecipeKind kind);

private:
	void rebuildPalette();

	void syncSessionPipeline();

	void syncSessionParams(bool skipPreviewReapply = false);

	void flushPipelineToSession(bool forApply = false);

	void runPreviewIfEnabled(bool showWarnings);

	void schedulePreviewRun(int delayMs, bool showWarnings = false);

	RobotInstruction::OpScope defaultScopeForNewOp() const;

	RobotInstruction::TrajectoryOpDescriptor makeDefaultOp(RobotInstruction::TrajectoryOpKind kind) const;

	void loadSelectedOpToParams();
	void loadSelectedOpToParamsImpl();

	void applyParamsToSelectedOp(bool skipPreviewReapply = false);

	void fillScopeFromUi(RobotInstruction::OpScope& scope) const;

	void syncScopeGroupFromTopBar();

	void refreshParamPanelForKind(RobotInstruction::TrajectoryOpKind kind);

	void refreshScopeFieldVisibility();

	void refreshUndoButtons();

	void syncUiAfterProgramRevision();

	bool reconcilePipelineScopes();

	void syncScopeComboFromSelectedOp();
	void syncGeometryBackendComboFromSelectedOp();
	void syncProjectBackendFromComboToPipeline(bool allProjectOps = false);
	std::vector<RobotInstruction::TrajectoryOpDescriptor> buildPipelineOpsForApply() const;

	void updateUiLabels();

	void refreshRawTrajectoryStatus();
	void setPipelineAppliedState(bool applied, bool announce = false);

	void showRawTrajectoryPreview(const RobotInstruction::RawTrajectory& traj, bool posesAlreadyWorldMm = false);

	std::string resolvePreviewBackendId(const RobotInstruction::RawTrajectory& traj) const;

	void onRawApplyRecipe();

	void onRawEmitProgram();

	void onProgramChanged(int index);

	void onGroupChanged(int index);

	void onPaletteDoubleClicked(QListWidgetItem* item);

	void onPipelineSelectionChanged(int index);

	void onPreviewToggled(bool checked);

	void onApplyClicked();

	void onResetClicked();

	void onUndoClicked();

	void onRedoClicked();

	void onRemovePipelineOpClicked();

	void onMovePipelineOpUpClicked();

	void onMovePipelineOpDownClicked();

	void onSaveTemplateClicked();

	void onLoadTemplateClicked();

	void showPipelineContextMenu(const QPoint& pos);

	void resetTrajectoryGenerationPages();

	bool m_useChinese = true;

	bool m_readOnly = false;

	RobotProgramStore* m_store = nullptr;

	ProgramEditService* m_editService = nullptr;

	TrajectoryEditSession* m_session = nullptr;
	TrajectoryEditObserver* m_observer = nullptr;

	RobotSimulationController* m_simController = nullptr;

	SimulationCommandWidget* m_commandPage = nullptr;

	IRobotMainWindowHost* m_host = nullptr;

	QGroupBox* m_rawGroupBox = nullptr;

	QLabel* m_rawStatusLabel = nullptr;

	QComboBox* m_rawRecipeCombo = nullptr;

	QPushButton* m_rawApplyBtn = nullptr;

	QPushButton* m_rawEmitBtn = nullptr;

	QLabel* m_programLabel = nullptr;

	QLabel* m_groupLabel = nullptr;

	QGroupBox* m_paramGroupBox = nullptr;

	QComboBox* m_programCombo = nullptr;

	QComboBox* m_groupCombo = nullptr;

	QListWidget* m_palette = nullptr;

	TrajectoryPipelineListWidget* m_pipeline = nullptr;

	QComboBox* m_scopeGroupCombo = nullptr;
	QComboBox* m_geometryBackendCombo = nullptr;
	QComboBox* m_nonRigidSourceCombo = nullptr;
	QComboBox* m_nonRigidTargetCombo = nullptr;
	QComboBox* m_externalTcpBackendCombo = nullptr;

	TrajectoryOpParamPanel* m_paramPanel = nullptr;

	QCheckBox* m_previewCheck = nullptr;

	QPushButton* m_applyBtn = nullptr;

	QPushButton* m_resetBtn = nullptr;

	QPushButton* m_undoBtn = nullptr;

	QPushButton* m_redoBtn = nullptr;

	QPushButton* m_saveTemplateBtn = nullptr;

	QPushButton* m_loadTemplateBtn = nullptr;

	bool m_loadingParams = false;
	bool m_flushingParams = false;
	bool m_pendingLoadSelectedOp = false;
	bool m_committingApply = false;
	bool m_pipelineAppliedSinceLastRawChange = false;
	int m_previewScheduleToken = 0;

	std::string m_selectedGroupId;
};

#endif // ROBOTWIDGET_TRAJECTORYEDITPAGEWIDGET_H
