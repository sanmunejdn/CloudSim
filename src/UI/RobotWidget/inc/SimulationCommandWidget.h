#pragma once

#include <json.hpp>

#include <QVector>
#include <QWidget>

#include "RobotInstructionModel.h"
#include "RobotSimulationTypes.h"
#include "robotwidget_global.h"

#include <memory>
#include <vector>

class QComboBox;
class QPushButton;
class QLabel;
class RobotProgramStore;
class InstructionProgramTreeWidget;

class ROBOTWIDGET_EXPORT SimulationCommandWidget : public QWidget
{
	Q_OBJECT

public:
	explicit SimulationCommandWidget(QWidget* parent = nullptr);

	void setProgramStore(RobotProgramStore* store);
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

	std::shared_ptr<RobotInstruction::Base> appendInstruction(RobotInstruction::Type type);
	bool appendInstructionFromJson(const nlohmann::json& j, std::string* errMsg = nullptr);

	void refreshInstructionList();
	void clearInstructionSelection();

signals:
	void robotSelectionChanged(int instanceIndex, const QString& sceneBackendId);
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

	QPushButton* createTypeButton(RobotInstruction::Type type);

	RobotProgramStore* m_programStore = nullptr;
	bool m_useChinese = false;
	bool m_simulationRunning = false;
	bool m_hasRobotContext = false;

	QLabel* m_hintLabel = nullptr;
	QComboBox* m_robotCombo = nullptr;
	QComboBox* m_tcpLinkCombo = nullptr;
	InstructionProgramTreeWidget* m_tree = nullptr;

	QVector<QPushButton*> m_typeButtons;
	QPushButton* m_removeBtn = nullptr;
	QPushButton* m_clearBtn = nullptr;
	QPushButton* m_runBtn = nullptr;
	QPushButton* m_stopBtn = nullptr;
	QPushButton* m_exportBtn = nullptr;
	QPushButton* m_tcpDragTeachBtn = nullptr;
	bool m_tcpDragTeachMode = false;
};
