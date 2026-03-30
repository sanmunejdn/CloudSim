#pragma once

#include <QVector>

#include <QWidget>

#include "RobotSimulationTypes.h"
#include "widget_global.h"

class QListWidget;

class QComboBox;

class QDoubleSpinBox;

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

signals:

	void runRequested();

	void stopRequested();

private:
	void rebuildCommandListWidget();

	void onAddClicked();

	void onRemoveClicked();

	void onClearClicked();

	void updateRunStopButtons();

	bool m_useChinese = false;
	bool m_simulationRunning = false;
	QLabel* m_hintLabel = nullptr;
	QComboBox* m_jointCombo = nullptr;

	QDoubleSpinBox* m_angleSpin = nullptr;

	QDoubleSpinBox* m_durationSpin = nullptr;

	QListWidget* m_list = nullptr;

	QPushButton* m_addBtn = nullptr;

	QPushButton* m_removeBtn = nullptr;

	QPushButton* m_clearBtn = nullptr;

	QPushButton* m_runBtn = nullptr;

	QPushButton* m_stopBtn = nullptr;

	QStringList m_jointNames;
	QVector<RobotSimulationCommand> m_commands;
};