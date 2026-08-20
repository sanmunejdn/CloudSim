/// @file RobotSimulationDockWidget.cpp
/// @brief 机器人仿真 Dock

#include "RobotSimulationDockWidget.h"

#include "DeviceCommandPageWidget.h"
#include "RobotAxisControlWidget.h"
#include "RobotCollisionSettingsWidget.h"
#include "RobotCommPageWidget.h"
#include "RobotExternalAxisSettingsWidget.h"
#include "RobotFrameSettingsWidget.h"
#include "SimulationCommandWidget.h"
#include "TrajectoryEditPageWidget.h"
#include "TrajectoryGenerationPageWidget.h"

#include <QAbstractButton>
#include <QButtonGroup>
#include <QFrame>
#include <QHBoxLayout>
#include <QPushButton>
#include <QScrollArea>
#include <QStackedWidget>
#include <QStyle>
#include <QVBoxLayout>

namespace
{
QScrollArea* wrapInScrollArea(QWidget* content, QWidget* parent)
{
	auto* scroll = new QScrollArea(parent);
	scroll->setWidget(content);
	scroll->setWidgetResizable(true);
	scroll->setFrameShape(QFrame::NoFrame);
	scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
	return scroll;
}

void polishBtnRole(QAbstractButton* btn, const char* role)
{
	if (!btn)
	{
		return;
	}
	btn->setProperty("btnRole", QLatin1String(role));
	if (btn->style())
	{
		btn->style()->unpolish(btn);
		btn->style()->polish(btn);
	}
}

void detachTabPage(QTabWidget* tabs, QWidget* page)
{
	if (!tabs || !page)
	{
		return;
	}
	const int idx = tabs->indexOf(page);
	if (idx >= 0)
	{
		tabs->removeTab(idx);
	}
}
} // namespace

RobotSimulationDockWidget::RobotSimulationDockWidget(QWidget* parent) : QWidget(parent)
{
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 0, 0, 0);
	root->setSpacing(0);

	auto* modeBar = new QWidget(this);
	auto* modeLayout = new QHBoxLayout(modeBar);
	modeLayout->setContentsMargins(8, 6, 8, 6);
	modeLayout->setSpacing(6);
	m_robotModeBtn = new QPushButton(modeBar);
	m_deviceModeBtn = new QPushButton(modeBar);
	m_robotModeBtn->setCheckable(true);
	m_deviceModeBtn->setCheckable(true);
	m_robotModeBtn->setMinimumHeight(28);
	m_deviceModeBtn->setMinimumHeight(28);
	polishBtnRole(m_robotModeBtn, "primary");
	polishBtnRole(m_deviceModeBtn, "secondary");
	m_modeGroup = new QButtonGroup(this);
	m_modeGroup->setExclusive(true);
	m_modeGroup->addButton(m_robotModeBtn, static_cast<int>(SimulationDockMode::Robot));
	m_modeGroup->addButton(m_deviceModeBtn, static_cast<int>(SimulationDockMode::CustomDevice));
	m_robotModeBtn->setChecked(true);
	modeLayout->addWidget(m_robotModeBtn, 1);
	modeLayout->addWidget(m_deviceModeBtn, 1);
	root->addWidget(modeBar);

	m_modeStack = new QStackedWidget(this);
	m_robotTabs = new QTabWidget(m_modeStack);
	m_robotTabs->setMovable(false);
	m_deviceTabs = new QTabWidget(m_modeStack);
	m_deviceTabs->setMovable(false);

	m_commandPage = new SimulationCommandWidget(m_robotTabs);
	m_axisPage = new RobotAxisControlWidget(this);
	m_axisTabHost = wrapInScrollArea(m_axisPage, this);
	m_axisPlaceholder = new QWidget(this);
	m_framePage = new RobotFrameSettingsWidget(m_robotTabs);
	m_externalAxisPage = new RobotExternalAxisSettingsWidget(m_robotTabs);
	m_collisionPage = new RobotCollisionSettingsWidget(m_robotTabs);
	m_generationPage = new TrajectoryGenerationPageWidget(m_robotTabs);
	m_trajectoryPage = new TrajectoryEditPageWidget(m_robotTabs);
	m_commPage = new RobotCommPageWidget(m_robotTabs);

	m_robotTabs->addTab(m_commandPage, QStringLiteral("Instructions"));
	m_robotTabs->addTab(m_axisTabHost, QStringLiteral("Axis"));
	m_robotTabs->addTab(wrapInScrollArea(m_framePage, m_robotTabs), QStringLiteral("Frames"));
	m_robotTabs->addTab(wrapInScrollArea(m_externalAxisPage, m_robotTabs), QStringLiteral("外部轴"));
	m_robotTabs->addTab(wrapInScrollArea(m_collisionPage, m_robotTabs), QStringLiteral("碰撞与规划"));
	m_robotTabs->addTab(wrapInScrollArea(m_generationPage, m_robotTabs), QStringLiteral("轨迹生成"));
	m_robotTabs->addTab(wrapInScrollArea(m_trajectoryPage, m_robotTabs), QStringLiteral("轨迹编辑"));
	m_robotTabs->addTab(wrapInScrollArea(m_commPage, m_robotTabs), QStringLiteral("机器人通讯"));

	m_deviceCommandPage = new DeviceCommandPageWidget(m_deviceTabs);
	m_deviceTabs->addTab(wrapInScrollArea(m_deviceCommandPage, m_deviceTabs), QStringLiteral("Device instructions"));
	m_deviceTabs->addTab(m_axisPlaceholder, QStringLiteral("Axis"));

	m_modeStack->addWidget(m_robotTabs);
	m_modeStack->addWidget(m_deviceTabs);
	root->addWidget(m_modeStack, 1);

	connect(m_modeGroup, QOverload<int>::of(&QButtonGroup::buttonClicked), this,
			&RobotSimulationDockWidget::onModeButtonClicked);

	retranslateUi();
	applyModeUi();
}

FeatureTrajectoryPageWidget* RobotSimulationDockWidget::featureTrajectoryPage() const
{
	return m_generationPage ? m_generationPage->brepPage() : nullptr;
}

MeshTrajectoryPageWidget* RobotSimulationDockWidget::meshTrajectoryPage() const
{
	return m_generationPage ? m_generationPage->meshPage() : nullptr;
}

void RobotSimulationDockWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	retranslateUi();
}

void RobotSimulationDockWidget::setDockMode(const SimulationDockMode mode)
{
	if (m_mode == mode)
	{
		return;
	}
	m_mode = mode;
	if (m_modeGroup)
	{
		if (QAbstractButton* btn = m_modeGroup->button(static_cast<int>(mode)))
		{
			btn->setChecked(true);
		}
	}
	applyModeUi();
	emit dockModeChanged(m_mode);
}

void RobotSimulationDockWidget::onModeButtonClicked(const int id)
{
	const auto mode = static_cast<SimulationDockMode>(id);
	if (m_mode == mode)
	{
		return;
	}
	m_mode = mode;
	applyModeUi();
	emit dockModeChanged(m_mode);
}

void RobotSimulationDockWidget::placeAxisTab()
{
	if (!m_axisTabHost || !m_axisPlaceholder || !m_robotTabs || !m_deviceTabs)
	{
		return;
	}
	const QString title = m_useChinese ? QStringLiteral("轴控制") : QStringLiteral("Axis");
	detachTabPage(m_robotTabs, m_axisTabHost);
	detachTabPage(m_robotTabs, m_axisPlaceholder);
	detachTabPage(m_deviceTabs, m_axisTabHost);
	detachTabPage(m_deviceTabs, m_axisPlaceholder);
	if (m_mode == SimulationDockMode::Robot)
	{
		m_robotTabs->insertTab(kTabIndexAxisControl, m_axisTabHost, title);
		m_deviceTabs->insertTab(kTabIndexDeviceAxisControl, m_axisPlaceholder, title);
	}
	else
	{
		m_deviceTabs->insertTab(kTabIndexDeviceAxisControl, m_axisTabHost, title);
		m_robotTabs->insertTab(kTabIndexAxisControl, m_axisPlaceholder, title);
	}
}

void RobotSimulationDockWidget::applyModeUi()
{
	const bool robot = m_mode == SimulationDockMode::Robot;
	placeAxisTab();
	if (m_axisPage)
	{
		m_axisPage->setReachableWorkspaceFeatureEnabled(robot);
	}
	if (m_modeStack)
	{
		m_modeStack->setCurrentIndex(robot ? 0 : 1);
	}
	polishBtnRole(m_robotModeBtn, robot ? "primary" : "secondary");
	polishBtnRole(m_deviceModeBtn, robot ? "secondary" : "primary");
}

void RobotSimulationDockWidget::retranslateUi()
{
	const auto t = [this](const QString& en, const QString& zh) { return m_useChinese ? zh : en; };
	if (m_robotModeBtn)
	{
		m_robotModeBtn->setText(t(QStringLiteral("Robot"), QStringLiteral("机器人")));
	}
	if (m_deviceModeBtn)
	{
		m_deviceModeBtn->setText(t(QStringLiteral("Custom device"), QStringLiteral("自定义设备")));
	}
	if (m_robotTabs)
	{
		m_robotTabs->setTabText(kTabIndexInstructions, t(QStringLiteral("Instructions"), QStringLiteral("指令")));
		m_robotTabs->setTabText(kTabIndexAxisControl, t(QStringLiteral("Axis"), QStringLiteral("轴控制")));
		m_robotTabs->setTabText(kTabIndexFrames, t(QStringLiteral("Frames"), QStringLiteral("坐标系")));
		m_robotTabs->setTabText(kTabIndexExternalAxes, t(QStringLiteral("External Axes"), QStringLiteral("外部轴")));
		m_robotTabs->setTabText(kTabIndexCollision,
								t(QStringLiteral("Collision & Planning"), QStringLiteral("碰撞与规划")));
		m_robotTabs->setTabText(kTabIndexTrajectoryGeneration,
								t(QStringLiteral("Trajectory Generation"), QStringLiteral("轨迹生成")));
		m_robotTabs->setTabText(kTabIndexTrajectoryEdit,
								t(QStringLiteral("Trajectory Edit"), QStringLiteral("轨迹编辑")));
		m_robotTabs->setTabText(kTabIndexRobotComm, t(QStringLiteral("Robot Comm"), QStringLiteral("机器人通讯")));
	}
	if (m_collisionPage)
	{
		m_collisionPage->setUseChinese(m_useChinese);
	}
	if (m_deviceTabs)
	{
		m_deviceTabs->setTabText(kTabIndexDeviceCommands,
								 t(QStringLiteral("Device instructions"), QStringLiteral("设备指令")));
		m_deviceTabs->setTabText(kTabIndexDeviceAxisControl, t(QStringLiteral("Axis"), QStringLiteral("轴控制")));
	}
}
