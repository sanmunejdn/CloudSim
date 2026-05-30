#pragma once



#include "TrajectoryPipelineTypes.h"

#include "robotwidget_global.h"



#include <QPoint>
#include <QWidget>

#include <string>



namespace RobotInstruction
{
struct RawTrajectory;
}



class QCheckBox;

class QComboBox;

class QGroupBox;

class QLabel;

class QPushButton;

class QListWidget;

class QListWidgetItem;

class TrajectoryPipelineListWidget;

class TrajectoryEditSession;

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



private:

	void rebuildPalette();

	void syncSessionPipeline();

	void syncSessionParams();

	void flushPipelineToSession();

	void runPreviewIfEnabled();

	RobotInstruction::OpScope defaultScopeForNewOp() const;

	RobotInstruction::TrajectoryOpDescriptor makeDefaultOp(RobotInstruction::TrajectoryOpKind kind) const;

	void loadSelectedOpToParams();
	void loadSelectedOpToParamsImpl();

	void applyParamsToSelectedOp();

	void fillScopeFromUi(RobotInstruction::OpScope& scope) const;

	void syncScopeGroupFromTopBar();

	void refreshParamPanelForKind(RobotInstruction::TrajectoryOpKind kind);

	void refreshScopeFieldVisibility();

	void refreshUndoButtons();

	void syncUiAfterProgramRevision();

	bool reconcilePipelineScopes();

	void syncScopeComboFromSelectedOp();

	void updateUiLabels();

	void refreshRawTrajectoryStatus();

	void showRawTrajectoryPreview(const RobotInstruction::RawTrajectory& traj);

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



	bool m_useChinese = true;

	bool m_readOnly = false;



	RobotProgramStore* m_store = nullptr;

	ProgramEditService* m_editService = nullptr;

	TrajectoryEditSession* m_session = nullptr;

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

	std::string m_selectedGroupId;

};

