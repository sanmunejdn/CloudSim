/// @file RobotSimulationController_robotComm.cpp
/// @brief 真实机器人通讯：Bridge 连接与场景镜像

#include "RobotSimulationController.h"

#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"
#include "IRobotMotionClient.h"
#include "RobotCommPageWidget.h"
#include "RobotSimulationDockWidget.h"
#include "SimulationCommandWidget.h"

#include <QStringList>

namespace
{
RobotCommBrand brandFromUi(const QString& brand)
{
	const QString b = brand.toLower();
	if (b == QLatin1String("abb"))
		return RobotCommBrand::Abb;
	if (b == QLatin1String("kuka"))
		return RobotCommBrand::Kuka;
	return RobotCommBrand::Fanuc;
}

QString formatJoints(const RobotFeedback& fb)
{
	if (!fb.hasJoints || fb.jointRad.empty())
		return QStringLiteral("Joints: -");
	QStringList parts;
	parts.reserve(static_cast<int>(fb.jointRad.size()));
	for (double r : fb.jointRad)
		parts << QString::number(r * 180.0 / 3.14159265358979323846, 'f', 2);
	return QStringLiteral("Joints(deg): %1").arg(parts.join(QLatin1Char(',')));
}

QString formatPose(const RobotFeedback& fb)
{
	if (!fb.hasPose)
		return QStringLiteral("TCP: -");
	return QStringLiteral("TCP(mm/deg): %1,%2,%3 / %4,%5,%6")
		.arg(fb.toolPoseInBase.positionMm[0], 0, 'f', 2)
		.arg(fb.toolPoseInBase.positionMm[1], 0, 'f', 2)
		.arg(fb.toolPoseInBase.positionMm[2], 0, 'f', 2)
		.arg(fb.toolPoseInBase.eulerDeg[0], 0, 'f', 2)
		.arg(fb.toolPoseInBase.eulerDeg[1], 0, 'f', 2)
		.arg(fb.toolPoseInBase.eulerDeg[2], 0, 'f', 2);
}

} // namespace

void RobotSimulationController::onRobotCommConnectRequested()
{
	RobotCommPageWidget* page = m_simulationDock ? m_simulationDock->robotCommPage() : nullptr;
	if (!page)
		return;

	if (!m_robotCommClient)
		m_robotCommClient = createRobotMotionClient();

	RobotCommBridgeEndpoint ep;
	ep.host = page->bridgeHost().toStdString();
	ep.port = static_cast<uint16_t>(page->bridgePort());
	ep.timeoutMs = 3000;

	if (m_host)
		m_host->appendRunInfo(QStringLiteral("[RobotComm] Connecting bridge %1:%2 ...")
								  .arg(page->bridgeHost())
								  .arg(page->bridgePort()));
	if (!m_robotCommClient->connectBridge(ep))
	{
		const QString msg = QStringLiteral("Bridge failed: %1")
								.arg(QString::fromStdString(m_robotCommClient->lastError()));
		page->setStatusText(msg);
		if (m_host)
			m_host->appendRunWarning(QStringLiteral("[RobotComm] %1").arg(msg));
		page->setConnectedUi(false, false);
		return;
	}

	RobotCommConnectConfig cfg;
	cfg.brand = brandFromUi(page->brand());
	cfg.robotHost = page->robotHost().toStdString();
	cfg.robotPort = static_cast<uint16_t>(page->robotPort());
	cfg.user = page->user().toStdString();
	cfg.password = page->password().toStdString();

	if (m_host)
		m_host->appendRunInfo(
			QStringLiteral("[RobotComm] Connecting robot %1 %2 ...").arg(page->brand(), page->robotHost()));
	if (!m_robotCommClient->connectRobot(cfg))
	{
		const QString msg = QStringLiteral("Robot failed: %1")
								.arg(QString::fromStdString(m_robotCommClient->lastError()));
		page->setStatusText(msg);
		if (m_host)
			m_host->appendRunWarning(QStringLiteral("[RobotComm] %1").arg(msg));
		page->setConnectedUi(true, false);
		return;
	}

	page->setStatusText(QStringLiteral("Connected"));
	if (m_host)
		m_host->appendRunInfo(QStringLiteral("[RobotComm] Connected OK"));
	page->setConnectedUi(true, true);
	m_robotCommMirror = page->mirrorEnabled();
	if (m_robotCommPollTimer)
	{
		m_robotCommPollTimer->setInterval(page->pollIntervalMs());
		m_robotCommPollTimer->start();
	}
}

void RobotSimulationController::onRobotCommDisconnectRequested()
{
	RobotCommPageWidget* page = m_simulationDock ? m_simulationDock->robotCommPage() : nullptr;
	if (m_robotCommPollTimer)
		m_robotCommPollTimer->stop();
	m_robotCommMirror = false;
	if (m_robotCommClient)
	{
		m_robotCommClient->disconnectRobot();
		m_robotCommClient->disconnectBridge();
	}
	if (page)
	{
		page->setStatusText(QStringLiteral("Disconnected"));
		page->setConnectedUi(false, false);
	}
	if (m_host)
		m_host->appendRunInfo(QStringLiteral("[RobotComm] Disconnected"));
}

void RobotSimulationController::onRobotCommMirrorToggled(bool enabled)
{
	m_robotCommMirror = enabled;
	if (m_host)
		m_host->appendRunInfo(enabled ? QStringLiteral("[RobotComm] Mirror ON")
									  : QStringLiteral("[RobotComm] Mirror OFF"));
}

void RobotSimulationController::onRobotCommPollIntervalChanged(int ms)
{
	if (m_robotCommPollTimer)
		m_robotCommPollTimer->setInterval(ms);
}

void RobotSimulationController::onRobotCommPollTick()
{
	RobotCommPageWidget* page = m_simulationDock ? m_simulationDock->robotCommPage() : nullptr;
	if (!m_robotCommClient || !m_robotCommClient->isBridgeConnected())
		return;

	RobotFeedback fb;
	if (!m_robotCommClient->getFeedback(fb))
	{
		if (page)
		{
			page->setStatusText(QStringLiteral("Feedback: %1")
									.arg(QString::fromStdString(m_robotCommClient->lastError())));
		}
		return;
	}

	if (page)
	{
		page->setFeedbackText(formatJoints(fb), formatPose(fb));
		page->setStatusText(QStringLiteral("Streaming"));
	}

	if (!m_robotCommMirror || !fb.hasJoints || fb.jointRad.empty())
		return;

	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	SimulationCommandWidget* cmd = m_simulationDock ? m_simulationDock->commandPage() : nullptr;
	if (!doc || !cmd)
		return;
	const int inst = cmd->currentRobotInstanceIndex();
	if (inst < 0)
		return;

	QVector<double> joints;
	joints.reserve(static_cast<int>(fb.jointRad.size()));
	for (double r : fb.jointRad)
		joints.push_back(r);

	QString err;
	if (!doc->applyJointAnglesRad(inst, joints, m_aggregatedJointAnglesRad, &err))
	{
		if (m_host)
			m_host->appendRunWarning(QStringLiteral("[RobotComm] Mirror apply failed: %1").arg(err));
	}
}
