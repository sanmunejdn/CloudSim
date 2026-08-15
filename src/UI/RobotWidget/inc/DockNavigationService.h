#ifndef ROBOTWIDGET_DOCKNAVIGATIONSERVICE_H
#define ROBOTWIDGET_DOCKNAVIGATIONSERVICE_H

/// @file DockNavigationService.h
/// @brief 仿真 Dock 机器人/设备模式与子 Tab 跳转

#include "robotwidget_global.h"

#include <QObject>

class RobotSimulationDockWidget;

class ROBOTWIDGET_EXPORT DockNavigationService : public QObject
{
	Q_OBJECT

public:
	explicit DockNavigationService(QObject* parent = nullptr);

	void setDock(RobotSimulationDockWidget* dock);

	void showRobotDockTab(int tabIndex);
	void showDeviceDockTab(int tabIndex);

private:
	RobotSimulationDockWidget* m_dock = nullptr;
};

#endif // ROBOTWIDGET_DOCKNAVIGATIONSERVICE_H
