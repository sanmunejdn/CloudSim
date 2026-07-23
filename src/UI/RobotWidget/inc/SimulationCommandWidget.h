#ifndef ROBOTWIDGET_SIMULATIONCOMMANDWIDGET_H
#define ROBOTWIDGET_SIMULATIONCOMMANDWIDGET_H

/// @file SimulationCommandWidget.h
/// @brief SimulationCommandWidget 接口

#include "robotwidget_global.h"

#include "RobotInstructionModel.h"
#include "RobotSimulationTypes.h"

#include <QVector>
#include <QWidget>
#include <memory>
#include <string>
#include <vector>

#include <json.hpp>

class QComboBox;

class QGroupBox;

class QPushButton;

class QLabel;

class QScrollArea;

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

	void setTcpDragTeachMode(bool enabled);

	bool tcpDragTeachMode() const;

	void setRobotInstances(const QStringList& labels, const QStringList& backendIds);

	int currentRobotInstanceIndex() const;

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

	void stopRequested();

	void exportProgramRequested();

	void tcpDragTeachModeChanged(bool enabled);

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

	QPushButton* createTypeButton(RobotInstruction::Type type);

	RobotProgramStore* m_programStore = nullptr;

	ProgramEditService* m_editService = nullptr;

	bool m_useChinese = true;

	bool m_simulationRunning = false;

	bool m_hasRobotContext = false;

	QGroupBox* m_instructionGroupBox = nullptr;

	QLabel* m_programLabel = nullptr;

	QGroupBox* m_functionGroupBox = nullptr;

	QComboBox* m_robotCombo = nullptr;

	QComboBox* m_tcpLinkCombo = nullptr;

	QComboBox* m_programCombo = nullptr;

	QPushButton* m_programNewBtn = nullptr;

	QPushButton* m_programRenameBtn = nullptr;

	QPushButton* m_programDeleteBtn = nullptr;

	InstructionProgramTreeWidget* m_tree = nullptr;

	QVector<QPushButton*> m_typeButtons;

	QPushButton* m_removeBtn = nullptr;

	QPushButton* m_clearBtn = nullptr;

	QPushButton* m_runBtn = nullptr;

	QPushButton* m_stopBtn = nullptr;

	QPushButton* m_exportBtn = nullptr;

	QPushButton* m_tcpDragTeachBtn = nullptr;

	bool m_tcpDragTeachMode = false;

	bool m_arcTeachPending = false;

	QScrollArea* m_scrollArea = nullptr;
};

#endif // ROBOTWIDGET_SIMULATIONCOMMANDWIDGET_H
