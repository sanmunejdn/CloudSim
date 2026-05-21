#pragma once

#include "robotwidget_global.h"

#include <QTabWidget>
#include <QWidget>

class SimulationCommandWidget;
class RobotAxisControlWidget;
class RobotFrameSettingsWidget;

/// Simulation dock: Instructions / Axis control / Frames tabs.
class ROBOTWIDGET_EXPORT RobotSimulationDockWidget : public QWidget
{
	Q_OBJECT

public:
	explicit RobotSimulationDockWidget(QWidget* parent = nullptr);

	QTabWidget* tabWidget() const { return m_tabs; }
	SimulationCommandWidget* commandPage() const { return m_commandPage; }
	RobotAxisControlWidget* axisPage() const { return m_axisPage; }
	RobotFrameSettingsWidget* framePage() const { return m_framePage; }

private:
	QTabWidget* m_tabs = nullptr;
	SimulationCommandWidget* m_commandPage = nullptr;
	RobotAxisControlWidget* m_axisPage = nullptr;
	RobotFrameSettingsWidget* m_framePage = nullptr;
};
