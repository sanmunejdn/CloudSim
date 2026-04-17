#pragma once

#include <QVector>

#include <QWidget>

#include "RobotInstructionModel.h"
#include "RobotSimulationTypes.h"
#include "widget_global.h"

#include <memory>
#include <vector>

class QListWidget;

class QComboBox;

class QPushButton;
class QLabel;

class WIDGET_EXPORT SimulationCommandWidget : public QWidget
{
	Q_OBJECT

public:
	explicit SimulationCommandWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setSimulationRunning(bool running);

	/// URDF revolute joint names (same order as joint indices). Empty disables Add/Run.
	void setRevoluteJointNames(const QStringList& names);

	/// Current instruction rows (in order).
	QVector<RobotSimulationCommand> commands() const;
	std::vector<std::shared_ptr<RobotInstruction::Base>> instructions(const QString& controllerId) const;
	void appendInstructionFromCurrentPose(
		RobotInstruction::Type type,
		const RobotInstruction::Vec3& poseMm,
		const RobotInstruction::Vec3& eulerDeg);
	void refreshInstructionList();
	void clearInstructionSelection();

signals:
	void addInstructionRequested(RobotInstruction::Type type);
	void instructionSelectionChanged(std::shared_ptr<RobotInstruction::Base> instruction);

	void runRequested();

	void stopRequested();

private:
	void rebuildCommandListWidget();

	void onAddClicked();

	void onRemoveClicked();

	void onClearClicked();

	void updateRunStopButtons();
	double instructionDurationSec(const RobotInstruction::Base& ins) const;

	bool m_useChinese = false;
	bool m_simulationRunning = false;
	bool m_hasRobotContext = false;
	QLabel* m_hintLabel = nullptr;
	QComboBox* m_typeCombo = nullptr;

	QListWidget* m_list = nullptr;

	QPushButton* m_addBtn = nullptr;

	QPushButton* m_removeBtn = nullptr;

	QPushButton* m_clearBtn = nullptr;

	QPushButton* m_runBtn = nullptr;

	QPushButton* m_stopBtn = nullptr;

	std::vector<std::shared_ptr<RobotInstruction::Base>> m_instructions;
};