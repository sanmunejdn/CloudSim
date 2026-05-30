#include "RobotSimulationDockWidget.h"

#include "RobotAxisControlWidget.h"
#include "RobotFrameSettingsWidget.h"
#include "SimulationCommandWidget.h"
#include "TrajectoryEditPageWidget.h"
#include "FeatureTrajectoryPageWidget.h"

#include <QVBoxLayout>

RobotSimulationDockWidget::RobotSimulationDockWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	m_tabs = new QTabWidget(this);
	layout->addWidget(m_tabs);
	m_commandPage = new SimulationCommandWidget(m_tabs);
	m_axisPage = new RobotAxisControlWidget(m_tabs);
	m_framePage = new RobotFrameSettingsWidget(m_tabs);
	m_trajectoryPage = new TrajectoryEditPageWidget(m_tabs);
	m_featurePage = new FeatureTrajectoryPageWidget(m_tabs);
	m_tabs->addTab(m_commandPage, QStringLiteral("Instructions"));
	m_tabs->addTab(m_axisPage, QStringLiteral("Axis control"));
	m_tabs->addTab(m_framePage, QStringLiteral("Frames"));
	m_tabs->addTab(m_featurePage, QStringLiteral("轨迹生成"));
	m_tabs->addTab(m_trajectoryPage, QStringLiteral("轨迹编辑"));
}
