#ifndef ROBOTWIDGET_ROBOTSIMULATIONDOCKWIDGET_H
#define ROBOTWIDGET_ROBOTSIMULATIONDOCKWIDGET_H

/// @file RobotSimulationDockWidget.h
/// @brief 仿真 Dock：模式切换 + 分模式子页（轴控跨模式单例 Tab）

#include "robotwidget_global.h"

#include <QTabWidget>
#include <QWidget>

class QAbstractButton;
class QButtonGroup;
class QStackedWidget;

class DeviceCommandPageWidget;
class FeatureTrajectoryPageWidget;
class MeshTrajectoryPageWidget;
class RobotAxisControlWidget;
class RobotCollisionSettingsWidget;
class RobotCommPageWidget;
class RobotExternalAxisSettingsWidget;
class RobotFrameSettingsWidget;
class SimulationCommandWidget;
class TrajectoryEditPageWidget;
class TrajectoryGenerationPageWidget;

enum class SimulationDockMode : int
{
	Robot = 0,
	CustomDevice = 1
};

class ROBOTWIDGET_EXPORT RobotSimulationDockWidget : public QWidget
{
	Q_OBJECT

public:
	/// 机器人子 Tab 索引（仅 robotTabWidget）；setMovable 后不保证与常量一致
	static constexpr int kTabIndexInstructions = 0;
	static constexpr int kTabIndexAxisControl = 1;
	static constexpr int kTabIndexFrames = 2;
	static constexpr int kTabIndexExternalAxes = 3;
	static constexpr int kTabIndexCollision = 4;
	static constexpr int kTabIndexTrajectoryGeneration = 5;
	static constexpr int kTabIndexTrajectoryEdit = 6;
	static constexpr int kTabIndexRobotComm = 7;

	/// 自定义设备子 Tab 索引（仅 deviceTabWidget）
	static constexpr int kTabIndexDeviceCommands = 0;
	/// 与机器人侧共用同一轴控实例
	static constexpr int kTabIndexDeviceAxisControl = 1;

	explicit RobotSimulationDockWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	SimulationDockMode dockMode() const { return m_mode; }
	void setDockMode(SimulationDockMode mode);

	/// 兼容旧调用：始终返回机器人子 Tab
	QTabWidget* tabWidget() const { return m_robotTabs; }
	QTabWidget* robotTabWidget() const { return m_robotTabs; }
	QTabWidget* deviceTabWidget() const { return m_deviceTabs; }

	SimulationCommandWidget* commandPage() const { return m_commandPage; }
	DeviceCommandPageWidget* deviceCommandPage() const { return m_deviceCommandPage; }
	RobotAxisControlWidget* axisPage() const { return m_axisPage; }
	RobotFrameSettingsWidget* framePage() const { return m_framePage; }
	RobotExternalAxisSettingsWidget* externalAxisPage() const { return m_externalAxisPage; }
	RobotCollisionSettingsWidget* collisionPage() const { return m_collisionPage; }
	TrajectoryEditPageWidget* trajectoryEditPage() const { return m_trajectoryPage; }
	FeatureTrajectoryPageWidget* featureTrajectoryPage() const;
	MeshTrajectoryPageWidget* meshTrajectoryPage() const;
	TrajectoryGenerationPageWidget* trajectoryGenerationPage() const { return m_generationPage; }
	RobotCommPageWidget* robotCommPage() const { return m_commPage; }

signals:
	void dockModeChanged(SimulationDockMode mode);

private slots:
	void onModeButtonClicked(int id);

private:
	void retranslateUi();
	void applyModeUi();
	void placeAxisTab();

	bool m_useChinese = true;
	SimulationDockMode m_mode = SimulationDockMode::Robot;

	QButtonGroup* m_modeGroup = nullptr;
	QAbstractButton* m_robotModeBtn = nullptr;
	QAbstractButton* m_deviceModeBtn = nullptr;

	RobotAxisControlWidget* m_axisPage = nullptr;
	QWidget* m_axisTabHost = nullptr;
	QWidget* m_axisPlaceholder = nullptr;
	QStackedWidget* m_modeStack = nullptr;
	QTabWidget* m_robotTabs = nullptr;
	QTabWidget* m_deviceTabs = nullptr;

	SimulationCommandWidget* m_commandPage = nullptr;
	DeviceCommandPageWidget* m_deviceCommandPage = nullptr;
	RobotFrameSettingsWidget* m_framePage = nullptr;
	RobotExternalAxisSettingsWidget* m_externalAxisPage = nullptr;
	RobotCollisionSettingsWidget* m_collisionPage = nullptr;
	TrajectoryEditPageWidget* m_trajectoryPage = nullptr;
	TrajectoryGenerationPageWidget* m_generationPage = nullptr;
	RobotCommPageWidget* m_commPage = nullptr;
};

#endif // ROBOTWIDGET_ROBOTSIMULATIONDOCKWIDGET_H
