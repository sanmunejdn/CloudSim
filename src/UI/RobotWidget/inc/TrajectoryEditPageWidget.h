#pragma once



#include "TrajectoryPipelineTypes.h"

#include "robotwidget_global.h"



#include <QWidget>



#include <string>

#include <vector>



class QComboBox;

class QDoubleSpinBox;

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

	void bindCommandPage(SimulationCommandWidget* commandPage);



	void refreshProgramAndGroupCombos();



private:

	void rebuildPalette();

	void syncSessionPipeline();

	void syncSessionParams();

	void flushPipelineToSession();

	RobotInstruction::OpScope defaultScopeForNewOp() const;

	RobotInstruction::TrajectoryOpDescriptor makeDefaultOp(RobotInstruction::TrajectoryOpKind kind) const;

	void loadSelectedOpToParams();

	void applyParamsToSelectedOp();

	void fillScopeFromUi(RobotInstruction::OpScope& scope) const;

	void syncScopeGroupFromTopBar();

	void refreshParamPanelForKind(RobotInstruction::TrajectoryOpKind kind);

	void refreshScopeFieldVisibility();

	void refreshUndoButtons();

	void syncUiAfterProgramRevision();

	void reconcilePipelineScopes();



	void refreshScopeKindCombo();

	void updateUiLabels();



	void onProgramChanged(int index);

	void onGroupChanged(int index);

	void onPaletteDoubleClicked(QListWidgetItem* item);

	void onPipelineSelectionChanged(int index);

	void onPreviewClicked();

	void onApplyClicked();

	void onResetClicked();

	void onUndoClicked();

	void onRedoClicked();

	void onRemovePipelineOpClicked();

	void onMovePipelineOpUpClicked();

	void onMovePipelineOpDownClicked();

	void onSaveTemplateClicked();

	void onLoadTemplateClicked();



	bool m_useChinese = true;

	bool m_readOnly = false;



	RobotProgramStore* m_store = nullptr;

	ProgramEditService* m_editService = nullptr;

	TrajectoryEditSession* m_session = nullptr;

	SimulationCommandWidget* m_commandPage = nullptr;



	QLabel* m_programLabel = nullptr;

	QLabel* m_groupLabel = nullptr;

	QGroupBox* m_paramGroupBox = nullptr;

	QLabel* m_scopeKindFieldLabel = nullptr;

	QLabel* m_scopeGroupFieldLabel = nullptr;

	QLabel* m_pointRangeFieldLabel = nullptr;

	QLabel* m_dxFieldLabel = nullptr;

	QLabel* m_dyFieldLabel = nullptr;

	QLabel* m_dzFieldLabel = nullptr;

	QLabel* m_axisXFieldLabel = nullptr;

	QLabel* m_axisYFieldLabel = nullptr;

	QLabel* m_axisZFieldLabel = nullptr;

	QLabel* m_angleFieldLabel = nullptr;

	QLabel* m_mirrorHintLabel = nullptr;



	QComboBox* m_programCombo = nullptr;

	QComboBox* m_groupCombo = nullptr;

	QListWidget* m_palette = nullptr;

	TrajectoryPipelineListWidget* m_pipeline = nullptr;



	QComboBox* m_scopeKindCombo = nullptr;

	QComboBox* m_scopeGroupCombo = nullptr;

	QDoubleSpinBox* m_pointFromSpin = nullptr;

	QDoubleSpinBox* m_pointToSpin = nullptr;

	QDoubleSpinBox* m_dxSpin = nullptr;

	QDoubleSpinBox* m_dySpin = nullptr;

	QDoubleSpinBox* m_dzSpin = nullptr;

	QDoubleSpinBox* m_axisXSpin = nullptr;

	QDoubleSpinBox* m_axisYSpin = nullptr;

	QDoubleSpinBox* m_axisZSpin = nullptr;

	QDoubleSpinBox* m_angleSpin = nullptr;



	QPushButton* m_previewBtn = nullptr;

	QPushButton* m_applyBtn = nullptr;

	QPushButton* m_resetBtn = nullptr;

	QPushButton* m_undoBtn = nullptr;

	QPushButton* m_redoBtn = nullptr;

	QPushButton* m_removeOpBtn = nullptr;

	QPushButton* m_moveUpBtn = nullptr;

	QPushButton* m_moveDownBtn = nullptr;

	QPushButton* m_saveTemplateBtn = nullptr;

	QPushButton* m_loadTemplateBtn = nullptr;



	bool m_loadingParams = false;

	std::string m_selectedGroupId;

};


