/// @file RobotSimulationController_controller.cpp
/// @brief ExternalController 模式：按实例多端口 listen + tick

#include "RobotSimulationController.h"

#include "ControllerManager.h"
#include "IRobotBackendPoseSink.h"
#include "IRobotDocumentHost.h"
#include "IRobotMainWindowHost.h"
#include "IRobotOsgViewHost.h"
#include "RobotAxisControlWidget.h"
#include "RobotSimulationTypes.h"
#include "SimulationCommandWidget.h"

#include <QTimer>

namespace
{
bool anyExternalEnabled(const std::vector<std::unique_ptr<ControllerManager>>& mgrs)
{
	for (const auto& m : mgrs)
	{
		if (m && m->isEnabled())
			return true;
	}
	return false;
}
} // namespace

void RobotSimulationController::onExternalControllerToggled(bool enabled)
{
	if (!m_externalControllerTimer)
	{
		m_externalControllerTimer = new QTimer(this);
		m_externalControllerTimer->setInterval(RobotSimulation::kPlaybackTimerIntervalMs);
		connect(m_externalControllerTimer, &QTimer::timeout, this,
				&RobotSimulationController::onExternalControllerTick);
	}

	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;

	if (enabled)
	{
		if (m_programExecutor.isRunning())
			stopRobotSimulation(false);

		for (auto& m : m_controllerManagers)
		{
			if (m)
			{
				m->setEnabled(false);
				m->stopListening();
			}
		}
		m_controllerManagers.clear();

		const int nInst = doc ? qMax(1, doc->robotKinematicInstanceCount()) : 1;
		QStringList ports;
		for (int i = 0; i < nInst; ++i)
		{
			auto mgr = std::make_unique<ControllerManager>(this);
			mgr->setDocument(doc);
			mgr->setBoundRobotInstanceIndex(i);
			mgr->setEnabled(true);
			const quint16 port = static_cast<quint16>(ControllerManager::kBasePort + i);
			if (!mgr->startListening(port))
			{
				const QString err = mgr->lastError();
				mgr->setEnabled(false);
				for (auto& m : m_controllerManagers)
				{
					if (m)
					{
						m->setEnabled(false);
						m->stopListening();
					}
				}
				m_controllerManagers.clear();
				if (m_host)
				{
					m_host->appendRunWarning(
						m_host->i18n(QStringLiteral("External controller listen failed: %1").arg(err),
									 QStringLiteral("外置控制器监听失败：%1").arg(err)));
				}
				if (m_host && m_host->simulationCommandPage())
					m_host->simulationCommandPage()->setExternalControllerChecked(false);
				return;
			}
			mgr->resetSimTime();
			ports << QString::number(port);
			m_controllerManagers.push_back(std::move(mgr));
		}

		m_externalControllerTimer->start();
		if (m_host)
		{
			m_host->appendRunInfo(m_host->i18n(
				QStringLiteral("External controller listening 127.0.0.1:[%1] (19620+instance)")
					.arg(ports.join(QLatin1Char(','))),
				QStringLiteral("外置控制器已监听 127.0.0.1:[%1]（端口=19620+实例）")
					.arg(ports.join(QLatin1Char(',')))));
		}
		if (m_host && m_host->simulationCommandPage())
			m_host->simulationCommandPage()->setExternalControllerChecked(true);
	}
	else
	{
		if (m_externalControllerTimer)
			m_externalControllerTimer->stop();
		for (auto& m : m_controllerManagers)
		{
			if (!m)
				continue;
			m->setEnabled(false);
			m->stopListening();
		}
		m_controllerManagers.clear();
		if (m_host && m_host->simulationCommandPage())
		{
			m_host->simulationCommandPage()->setExternalControllerChecked(false);
			m_host->simulationCommandPage()->setExternalControllerClientConnected(false);
		}
	}
}

void RobotSimulationController::onExternalControllerTick()
{
	if (!anyExternalEnabled(m_controllerManagers))
		return;

	IRobotDocumentHost* doc = m_host ? m_host->document() : nullptr;
	IRobotOsgViewHost* osg = m_host ? m_host->osgView() : nullptr;
	IRobotBackendPoseSink* poseSink = doc ? doc->poseSink() : nullptr;

	bool anyClient = false;
	bool anyApplied = false;
	for (auto& mgr : m_controllerManagers)
	{
		if (!mgr || !mgr->isEnabled())
			continue;
		mgr->setDocument(doc);
		mgr->pollIncoming();
		anyClient = anyClient || mgr->hasClient();
		if (mgr->tickApply(poseSink, m_aggregatedJointAnglesRad))
		{
			anyApplied = true;
			const int instIdx = mgr->boundRobotInstanceIndex();
			if (doc && m_host && m_host->robotAxisControlPage() && instIdx >= 0)
			{
				const int uiInst =
					(m_host->simulationCommandPage() &&
					 m_host->simulationCommandPage()->currentRobotInstanceIndex() >= 0)
						? m_host->simulationCommandPage()->currentRobotInstanceIndex()
						: instIdx;
				if (uiInst == instIdx)
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
			}
		}
	}

	if (m_host && m_host->simulationCommandPage())
		m_host->simulationCommandPage()->setExternalControllerClientConnected(anyClient);

	if (anyApplied && osg)
		osg->requestRedraw();
}
