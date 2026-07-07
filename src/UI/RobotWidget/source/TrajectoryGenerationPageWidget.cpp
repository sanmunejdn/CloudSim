#include "TrajectoryGenerationPageWidget.h"

#include "FeatureTrajectoryPageWidget.h"
#include "MeshTrajectoryPageWidget.h"

#include <QTabWidget>
#include <QVBoxLayout>

TrajectoryGenerationPageWidget::TrajectoryGenerationPageWidget(QWidget* parent)
	: QWidget(parent)
{
	auto* layout = new QVBoxLayout(this);
	layout->setContentsMargins(0, 0, 0, 0);
	m_tabs = new QTabWidget(this);
	layout->addWidget(m_tabs);
	m_brepPage = new FeatureTrajectoryPageWidget(m_tabs);
	m_meshPage = new MeshTrajectoryPageWidget(m_tabs);
	m_tabs->addTab(m_brepPage, QStringLiteral("CAD/BREP"));
	m_tabs->addTab(m_meshPage, QStringLiteral("Mesh"));
}

void TrajectoryGenerationPageWidget::setUseChinese(const bool chinese)
{
	m_brepPage->setUseChinese(chinese);
	m_meshPage->setUseChinese(chinese);
	m_tabs->setTabText(0, chinese ? QStringLiteral("CAD/BREP") : QStringLiteral("CAD/BREP"));
	m_tabs->setTabText(1, chinese ? QStringLiteral("Mesh") : QStringLiteral("Mesh"));
}

void TrajectoryGenerationPageWidget::bindHost(IRobotMainWindowHost* host)
{
	m_brepPage->bindHost(host);
	m_meshPage->bindHost(host);
}

void TrajectoryGenerationPageWidget::bindSession(TrajectoryEditSession* session)
{
	m_brepPage->bindSession(session);
	m_meshPage->bindSession(session);
}

void TrajectoryGenerationPageWidget::bindSimulationController(RobotSimulationController* controller)
{
	m_brepPage->bindSimulationController(controller);
	m_meshPage->bindSimulationController(controller);
}

void TrajectoryGenerationPageWidget::setStepPathResolver(
	std::function<QString(const QString& backendId)> resolver)
{
	m_brepPage->setStepPathResolver(std::move(resolver));
}
