/// @file DockNavigationService.cpp
/// @brief DockNavigationService 实现

#include "DockNavigationService.h"

#include "RobotSimulationDockWidget.h"

DockNavigationService::DockNavigationService(QObject* parent) : QObject(parent) {}

void DockNavigationService::setDock(RobotSimulationDockWidget* dock)
{
	m_dock = dock;
}

void DockNavigationService::showRobotDockTab(const int tabIndex)
{
	if (!m_dock || !m_dock->robotTabWidget())
	{
		return;
	}
	m_dock->setDockMode(SimulationDockMode::Robot);
	m_dock->robotTabWidget()->setCurrentIndex(tabIndex);
}

void DockNavigationService::showDeviceDockTab(const int tabIndex)
{
	if (!m_dock || !m_dock->deviceTabWidget())
	{
		return;
	}
	m_dock->setDockMode(SimulationDockMode::CustomDevice);
	m_dock->deviceTabWidget()->setCurrentIndex(tabIndex);
}
