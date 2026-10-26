#ifndef ROBOTWIDGET_SIMULATIONCOMMANDWIDGET_H
#define ROBOTWIDGET_SIMULATIONCOMMANDWIDGET_H

/// @file SimulationCommandWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief SimulationCommandWidget 接口

#include "robotwidget_global.h"

#include "CeeRule.h"
#include "PlcSignalMapTypes.h"
#include "RobotInstructionModel.h"
#include "RobotSimulationTypes.h"

#include <QVector>
#include <QWidget>
#include <memory>
#include <string>
#include <vector>

#include <json.hpp>

class QAction;

class QComboBox;

class QPushButton;

class QLabel;

class QCheckBox;

class QListWidget;
class QLineEdit;

class ExternalControllerDialog;

class RobotProgramStore;

class InstructionProgramTreeWidget;

class ProgramEditService;

class ROBOTWIDGET_EXPORT SimulationCommandWidget : public QWidget

{
	Q_OBJECT

public:
	explicit SimulationCommandWidget(QWidget* parent = nullptr);

	void setProgramStore(RobotProgramStore* store);

	void setProgramEditService(ProgramEditService* service);

	void bindProgramTree();

	void setUseChinese(bool chinese);

	void setSimulationRunning(bool running);

	void setRunModeIndex(int index);
	int runModeIndex() const;
	void refreshCeeRulesList(const QVector<CeeRule>& rules);
	void setEventLogicAuthorityIndex(int index);
	int eventLogicAuthorityIndex() const;
	void setExternalPlcLinkConfig(const ExternalPlcLinkConfig& config);
	void setPlcSignalMap(const QVector<PlcSignalMapEntry>& map);

	/// 当前仿真播放倍率（预设下拉）
	double playbackRate() const;

	void setTcpDragTeachMode(bool enabled);

	bool tcpDragTeachMode() const;

	void setInstructionWaypointPickMode(bool enabled);

	bool instructionWaypointPickMode() const;

	void setRobotInstances(const QStringList& labels, const QStringList& backendIds);

	int currentRobotInstanceIndex() const;

	void setCurrentRobotInstanceIndex(int instanceIndex);

	/// 按程序 id 或显示名切换当前程序；找不到返回 false
	bool selectProgramByIdOrName(const QString& programIdOrName);

	QString currentRobotBackendId() const;

	void setRevoluteJointNames(const QStringList& names);

	void setTcpLinkOptions(const QStringList& linkNames, const QString& preferredLink);

	QString selectedTcpLink() const;

	QVector<RobotSimulationCommand> commands() const;

	std::vector<std::shared_ptr<RobotInstruction::Base>> instructions(const QString& robotBackendId) const;

	std::vector<std::shared_ptr<RobotInstruction::Base>> instructionList() const;

	std::shared_ptr<RobotInstruction::Base> appendInstructionFromCurrentPose(

		RobotInstruction::Type type,

		const RobotInstruction::Vec3& poseMm,

		const RobotInstruction::Vec3& eulerDeg,

		bool deferInstructionSelection = false);

	std::shared_ptr<RobotInstruction::Base> appendArcInstructionFromPoses(

		const RobotInstruction::Vec3& viaPoseMm, const RobotInstruction::Vec3& viaEulerDeg,
		const RobotInstruction::Vec3& endPoseMm, const RobotInstruction::Vec3& endEulerDeg,
		bool deferInstructionSelection = false);

	void setArcTeachPending(bool pending);

	bool arcTeachPending() const { return m_arcTeachPending; }

	std::shared_ptr<RobotInstruction::Base> appendInstruction(RobotInstruction::Type type);

	bool appendInstructionFromJson(const nlohmann::json& j, std::string* errMsg = nullptr);

	std::vector<std::string> selectedMotionInstructionIds() const;

	void refreshInstructionList();

	void clearInstructionSelection();

	InstructionProgramTreeWidget* instructionTree() const { return m_tree; }

signals:

	void robotSelectionChanged(int instanceIndex, const QString& sceneBackendId);

	void activeProgramChanged(const QString& programIdUtf8);

	void groupsChanged();

	void addInstructionRequested(RobotInstruction::Type type);

	void instructionSelectionChanged(std::shared_ptr<RobotInstruction::Base> instruction);

	void runRequested();

	/// 全部机器人实例 + 设备程序统一 Run
	void runAllRequested();

	void stopRequested();

	/// 0=TimeBased，1=EventBased
	void runModeChanged(int modeIndex);

	void syncCeeRulesFromBindingsRequested();
	void ceeRulesEdited(const QVector<CeeRule>& rules);
	/// 0=LocalCee，1=ExternalPlc
	void eventLogicAuthorityChanged(int authorityIndex);
	void externalPlcConnectRequested();
	void externalPlcDisconnectRequested();
	void externalPlcLinkEdited(const ExternalPlcLinkConfig& config);
	void plcSignalMapEdited(const QVector<PlcSignalMapEntry>& map);

	void exportProgramRequested();

	void playbackRateChanged(double rate);

	/// ExternalController：localhost TCP 外置控制器（与指令回放互斥）
	void externalControllerToggled(bool enabled);

	/// 0=FromInstruction（链式），1=FromCurrentPose
	void ikSeedPolicyChanged(int policy);

	/// 会话级：允许 SoftAccepted 姿态
	void allowApproximateOrientationChanged(bool enabled);

	/// 勾选后在三维视图绘制 TCP 实际走过的轨迹
	void tcpMotionTrailToggled(bool enabled);

	void tcpDragTeachModeChanged(bool enabled);

	void instructionWaypointPickModeChanged(bool enabled);

public:
	/// 与设置对话框双向同步 Host 监听勾选（失败回滚时用）
	void setExternalControllerChecked(bool checked);

	void setExternalControllerClientConnected(bool connected);

	bool isExternalControllerChecked() const;

private:
	void rebuildCommandListWidget();

	void onAddTypeButtonClicked();

	void onRemoveClicked();

	void onClearClicked();

	void onRobotComboChanged(int index);

	void updateRunStopButtons();

	void updateTypeButtonLabels();

	void setTypeButtonsEnabled(bool enabled);

	double instructionDurationSec(const RobotInstruction::Base& ins) const;

	void requestAddInstruction(RobotInstruction::Type type);

	void refreshProgramCombo();

	void onProgramComboChanged(int index);

	void onProgramNewClicked();

	void onProgramRenameClicked();

	void onProgramDeleteClicked();

	void onCreateGroupRequested(const QString& name, const std::vector<std::string>& memberIds);

	void onDissolveGroupRequested(const std::string& groupId);

	void onRenameGroupRequested(const std::string& groupId, const QString& newName);

	void updateProgramGroupUi();

	void refreshArcTeachButtonLabels();

	void updateTcpDragTeachUi(bool enabled);

	void updateWaypointPickUi(bool enabled);

	void refreshEditModeHint();

	void openExternalControllerSettings();

	void onExternalControllerDialogListenToggled(bool enabled);

	void updateCeeRulesUiVisibility();
	void rebuildCeeRulesListWidget();
	void onCeeAddRuleClicked();
	void onCeeRemoveRuleClicked();
	void emitExternalPlcLinkFromUi();
	void onPlcMapAddClicked();
	void onPlcMapRemoveClicked();

	QPushButton* createTypeButton(RobotInstruction::Type type);

	RobotProgramStore* m_programStore = nullptr;

	ProgramEditService* m_editService = nullptr;

	bool m_useChinese = true;

	bool m_simulationRunning = false;

	bool m_hasRobotContext = false;

	QLabel* m_robotLabel = nullptr;

	QLabel* m_programLabel = nullptr;

	QLabel* m_insertLabel = nullptr;

	QLabel* m_editLabel = nullptr;

	QComboBox* m_robotCombo = nullptr;

	/// 界面隐藏；setTcpLinkOptions / selectedTcpLink 仍供规划回退
	QComboBox* m_tcpLinkCombo = nullptr;

	QComboBox* m_programCombo = nullptr;

	QPushButton* m_programMoreBtn = nullptr;

	QAction* m_programNewAct = nullptr;

	QAction* m_programRenameAct = nullptr;

	QAction* m_programDeleteAct = nullptr;

	InstructionProgramTreeWidget* m_tree = nullptr;

	QVector<QPushButton*> m_typeButtons;

	QPushButton* m_removeBtn = nullptr;

	QPushButton* m_clearBtn = nullptr;

	QPushButton* m_runBtn = nullptr;
	QPushButton* m_runAllBtn = nullptr;

	QPushButton* m_stopBtn = nullptr;

	QLabel* m_runModeLabel = nullptr;
	QComboBox* m_runModeCombo = nullptr;
	QListWidget* m_ceeRulesList = nullptr;
	QPushButton* m_ceeSyncBindingsBtn = nullptr;
	QPushButton* m_ceeAddRuleBtn = nullptr;
	QPushButton* m_ceeRemoveRuleBtn = nullptr;
	QVector<CeeRule> m_ceeRulesCache;
	QLabel* m_authorityLabel = nullptr;
	QComboBox* m_authorityCombo = nullptr;
	QLineEdit* m_plcGatewayEdit = nullptr;
	QLineEdit* m_plcPortEdit = nullptr;
	QLineEdit* m_plcUnitEdit = nullptr;
	QPushButton* m_plcConnectBtn = nullptr;
	QPushButton* m_plcDisconnectBtn = nullptr;
	QListWidget* m_plcMapList = nullptr;
	QPushButton* m_plcMapAddBtn = nullptr;
	QPushButton* m_plcMapRemoveBtn = nullptr;
	QVector<PlcSignalMapEntry> m_plcMapCache;

	QLabel* m_playbackRateLabel = nullptr;

	QComboBox* m_playbackRateCombo = nullptr;

	QCheckBox* m_externalControllerCheck = nullptr;

	QPushButton* m_externalControllerSettingsBtn = nullptr;

	ExternalControllerDialog* m_externalControllerDialog = nullptr;

	QLabel* m_ikSeedLabel = nullptr;

	QComboBox* m_ikSeedCombo = nullptr;

	QCheckBox* m_allowApproxOrientCheck = nullptr;

	QCheckBox* m_tcpMotionTrailCheck = nullptr;

	QPushButton* m_exportBtn = nullptr;

	QPushButton* m_tcpDragTeachBtn = nullptr;

	QPushButton* m_waypointPickBtn = nullptr;

	QLabel* m_tcpDragHintLabel = nullptr;

	bool m_tcpDragTeachMode = false;

	bool m_waypointPickMode = false;

	bool m_arcTeachPending = false;
};

#endif // ROBOTWIDGET_SIMULATIONCOMMANDWIDGET_H
