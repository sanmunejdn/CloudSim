#ifndef ROBOTWIDGET_ROBOTSIMULATIONDOCKWIDGET_H
#define ROBOTWIDGET_ROBOTSIMULATIONDOCKWIDGET_H

/// @file RobotSimulationDockWidget.h
/// @brief 仿真 Dock：指令/轴控制/坐标系/外部轴/轨迹/机器人通讯页签

#include "robotwidget_global.h"

#include "RobotAxisControlWidget.h"
#include "RobotCollisionSettingsWidget.h"
#include "RobotCommPageWidget.h"
#include "RobotExternalAxisSettingsWidget.h"
#include "RobotFrameSettingsWidget.h"
#include "SimulationCommandWidget.h"
#include "TrajectoryEditPageWidget.h"
#include "TrajectoryGenerationPageWidget.h"

#include <QTabWidget>
#include <QWidget>

class ROBOTWIDGET_EXPORT RobotSimulationDockWidget : public QWidget
{
	Q_OBJECT

public:
	static constexpr int kTabIndexInstructions = 0;
	static constexpr int kTabIndexAxisControl = 1;
	static constexpr int kTabIndexFrames = 2;
	static constexpr int kTabIndexExternalAxes = 3;
	static constexpr int kTabIndexCollision = 4;
	static constexpr int kTabIndexTrajectoryGeneration = 5;
	static constexpr int kTabIndexTrajectoryEdit = 6;
	static constexpr int kTabIndexRobotComm = 7;

	explicit RobotSimulationDockWidget(QWidget* parent = nullptr);

	QTabWidget* tabWidget() const { return m_tabs; }
	SimulationCommandWidget* commandPage() const { return m_commandPage; }
	RobotAxisControlWidget* axisPage() const { return m_axisPage; }
	RobotFrameSettingsWidget* framePage() const { return m_framePage; }
	RobotExternalAxisSettingsWidget* externalAxisPage() const { return m_externalAxisPage; }
	RobotCollisionSettingsWidget* collisionPage() const { return m_collisionPage; }
	TrajectoryEditPageWidget* trajectoryEditPage() const { return m_trajectoryPage; }
	FeatureTrajectoryPageWidget* featureTrajectoryPage() const
	{
		return m_generationPage ? m_generationPage->brepPage() : nullptr;
	}
	MeshTrajectoryPageWidget* meshTrajectoryPage() const
	{
		return m_generationPage ? m_generationPage->meshPage() : nullptr;
	}
	TrajectoryGenerationPageWidget* trajectoryGenerationPage() const { return m_generationPage; }
	RobotCommPageWidget* robotCommPage() const { return m_commPage; }

private:
	QTabWidget* m_tabs = nullptr;
	SimulationCommandWidget* m_commandPage = nullptr;
	RobotAxisControlWidget* m_axisPage = nullptr;
	RobotFrameSettingsWidget* m_framePage = nullptr;
	RobotExternalAxisSettingsWidget* m_externalAxisPage = nullptr;
	RobotCollisionSettingsWidget* m_collisionPage = nullptr;
	TrajectoryEditPageWidget* m_trajectoryPage = nullptr;
	TrajectoryGenerationPageWidget* m_generationPage = nullptr;
	RobotCommPageWidget* m_commPage = nullptr;
};

#endif // ROBOTWIDGET_ROBOTSIMULATIONDOCKWIDGET_H
