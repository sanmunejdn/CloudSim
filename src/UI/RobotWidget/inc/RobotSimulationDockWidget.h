#pragma once

#include "robotwidget_global.h"

#include <QTabWidget>
#include <QWidget>

class SimulationCommandWidget;
class RobotAxisControlWidget;
class RobotFrameSettingsWidget;
class TrajectoryEditPageWidget;
class FeatureTrajectoryPageWidget;

/// 仿真 Dock：指令/轴控制/坐标系/轨迹生成/轨迹编辑页签
class ROBOTWIDGET_EXPORT RobotSimulationDockWidget : public QWidget
{
	Q_OBJECT

public:
	static constexpr int kTabIndexInstructions = 0;
	static constexpr int kTabIndexAxisControl = 1;
	static constexpr int kTabIndexFrames = 2;
	static constexpr int kTabIndexTrajectoryGeneration = 3;
	static constexpr int kTabIndexTrajectoryEdit = 4;

	explicit RobotSimulationDockWidget(QWidget* parent = nullptr);

	QTabWidget* tabWidget() const { return m_tabs; }
	SimulationCommandWidget* commandPage() const { return m_commandPage; }
	RobotAxisControlWidget* axisPage() const { return m_axisPage; }
	RobotFrameSettingsWidget* framePage() const { return m_framePage; }
	TrajectoryEditPageWidget* trajectoryEditPage() const { return m_trajectoryPage; }
	FeatureTrajectoryPageWidget* featureTrajectoryPage() const { return m_featurePage; }

private:
	QTabWidget* m_tabs = nullptr;
	SimulationCommandWidget* m_commandPage = nullptr;
	RobotAxisControlWidget* m_axisPage = nullptr;
	RobotFrameSettingsWidget* m_framePage = nullptr;
	TrajectoryEditPageWidget* m_trajectoryPage = nullptr;
	FeatureTrajectoryPageWidget* m_featurePage = nullptr;
};
