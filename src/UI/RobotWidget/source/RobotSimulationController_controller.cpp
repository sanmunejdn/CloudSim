/// @file RobotSimulationController_controller.cpp
/// @brief ExternalController 模式：ControllerManager listen + tick

#include "RobotSimulationController.h"

#include "IRobotBackendPoseSink.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotAxisControlWidget.h"
#include "RobotSimulationTypes.h"
#include "SimulationCommandWidget.h"

#include <QTimer>

void RobotSimulationController::onExternalControllerToggled(bool enabled)
{
	if (!m_controllerManager)
		m_controllerManager = std::make_unique<ControllerManager>(this);

	if (!m_externalControllerTimer)
	{
		m_externalControllerTimer = new QTimer(this);
		m_externalControllerTimer->setInterval(RobotSimulation::kPlaybackTimerIntervalMs);
		connect(m_externalControllerTimer, &QTimer::timeout, this,
				&RobotSimulationController::onExternalControllerTick);
	}

	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	m_controllerManager->setDocument(doc);

	if (enabled)
	{
		// 与指令回放互斥
		if (m_programExecutor.isRunning())
			stopRobotSimulation(false);

		const int instIdx =
			(m_host && m_host->simulationCommandPage() &&
			 m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0)
				? m_host->simulationCommandPage()->currentRobotInstanceIndex()
				: 0;
		m_controllerManager->context().setRobotInstanceIndex(instIdx);
		m_controllerManager->setEnabled(true);
		if (!m_controllerManager->startListening(19620))
		{
			if (m_host)
			{
				m_host->appendRunWarning(
					m_host->i18n(QStringLiteral("External controller listen failed: %1")
									 .arg(m_controllerManager->lastError()),
								 QStringLiteral("外置控制器监听失败：%1")
									 .arg(m_controllerManager->lastError())));
			}
			m_controllerManager->setEnabled(false);
			if (m_host && m_host->simulationCommandPage())
				m_host->simulationCommandPage()->setExternalControllerChecked(false);
			return;
		}
		m_controllerManager->resetSimTime();
		m_externalControllerTimer->start();
		if (m_host)
		{
			m_host->appendRunInfo(
				m_host->i18n(QStringLiteral("External controller listening on 127.0.0.1:19620"),
							 QStringLiteral("外置控制器已监听 127.0.0.1:19620")));
		}
		if (m_host && m_host->simulationCommandPage())
			m_host->simulationCommandPage()->setExternalControllerChecked(true);
	}
	else
	{
		if (m_externalControllerTimer)
			m_externalControllerTimer->stop();
		m_controllerManager->setEnabled(false);
		m_controllerManager->stopListening();
		if (m_host && m_host->simulationCommandPage())
		{
			m_host->simulationCommandPage()->setExternalControllerChecked(false);
			m_host->simulationCommandPage()->setExternalControllerClientConnected(false);
		}
	}
}

void RobotSimulationController::onExternalControllerTick()
{
	if (!m_controllerManager || !m_controllerManager->isEnabled())
		return;

	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;
	m_controllerManager->setDocument(doc);

	m_controllerManager->pollIncoming();
	if (m_host && m_host->simulationCommandPage())
		m_host->simulationCommandPage()->setExternalControllerClientConnected(
			m_controllerManager->hasClient());

	if (m_controllerManager->tickApply(poseSink, m_aggregatedJointAnglesRad))
	{
		// 轴控页不订阅场景 FK，需把聚合关节写回滑条（Silent 避免再触发 setJoint→FK）
		const int instIdx =
			(m_host && m_host->simulationCommandPage() &&
			 m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0)
				? m_host->simulationCommandPage()->currentRobotInstanceIndex()
				: m_controllerManager->context().robotInstanceIndex();
		if (doc && m_host && m_host->robotAxisControlPage() && instIdx >= 0)
		{
			const int nj = doc->robotRevoluteJointCountForInstance(instIdx);
			const int jointOffset = doc->robotJointOffsetInAggregatedVector(instIdx);
			if (nj > 0 && m_aggregatedJointAnglesRad.size() >= jointOffset + nj &&
				m_host->robotAxisControlPage()->jointCount() == nj)
			{
				m_host->robotAxisControlPage()->setJointAnglesRadSilent(
					m_aggregatedJointAnglesRad.mid(jointOffset, nj));
			}
		}
		if (osg)
			osg->requestRedraw();
	}
}
