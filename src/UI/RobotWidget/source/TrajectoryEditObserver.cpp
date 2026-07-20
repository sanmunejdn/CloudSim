/// @file TrajectoryEditObserver.cpp
/// @brief TrajectoryEditObserver 实现

#include "TrajectoryEditObserver.h"

#include "ProgramEditService.h"
#include "TrajectoryEditSession.h"

TrajectoryEditObserver::TrajectoryEditObserver(QObject* parent) : QObject(parent) {}

void TrajectoryEditObserver::bindSession(TrajectoryEditSession* session)
{
	m_session = session;
}

void TrajectoryEditObserver::bindEditService(ProgramEditService* service)
{
	m_editService = service;
}

bool TrajectoryEditObserver::syncDraftOps(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	m_draftOps = ops;
	if (!m_session)
	{
		return false;
	}
	m_session->updatePipelineOps(ops, false);
	return m_session->syncPipelineEngine(ops);
}

void TrajectoryEditObserver::loadPipeline(const std::vector<RobotInstruction::TrajectoryOpDescriptor>& ops)
{
	if (!m_session)
	{
		return;
	}
	m_draftOps = ops;
	m_session->setPipeline(ops);
	m_session->syncPipelineEngine(ops);
	emit pipelineStructureChanged();
}

void TrajectoryEditObserver::updateNodeParams(const std::size_t nodeIndex,
											  const RobotInstruction::TrajectoryOpDescriptor& descriptor)
{
	if (!m_session || nodeIndex >= m_draftOps.size())
	{
		return;
	}
	m_draftOps[nodeIndex] = descriptor;
	m_session->updatePipelineOps(m_draftOps, false);
	m_session->syncPipelineEngine(m_draftOps);
	if (m_session->hasRawTrajectory())
	{
		QString err;
		m_session->runPipelineEngineFrom(nodeIndex, &err);
	}
	emit previewRequested();
}

void TrajectoryEditObserver::moveNodeUp(const std::size_t nodeIndex)
{
	if (nodeIndex == 0 || nodeIndex >= m_draftOps.size())
	{
		return;
	}
	std::swap(m_draftOps[nodeIndex - 1], m_draftOps[nodeIndex]);
	syncDraftOps(m_draftOps);
	emit pipelineStructureChanged();
	emit previewRequested();
}

void TrajectoryEditObserver::moveNodeDown(const std::size_t nodeIndex)
{
	if (nodeIndex + 1 >= m_draftOps.size())
	{
		return;
	}
	std::swap(m_draftOps[nodeIndex], m_draftOps[nodeIndex + 1]);
	syncDraftOps(m_draftOps);
	emit pipelineStructureChanged();
	emit previewRequested();
}

void TrajectoryEditObserver::removeNode(const std::size_t nodeIndex)
{
	if (nodeIndex >= m_draftOps.size())
	{
		return;
	}
	m_draftOps.erase(m_draftOps.begin() + static_cast<std::ptrdiff_t>(nodeIndex));
	syncDraftOps(m_draftOps);
	emit pipelineStructureChanged();
	emit previewRequested();
}

bool TrajectoryEditObserver::preview(QString* outError)
{
	if (!m_session)
	{
		if (outError)
		{
			*outError = QStringLiteral("session not bound");
		}
		return false;
	}
	if (m_session->hasRawTrajectory())
	{
		return runPreviewIfRaw(outError);
	}
	return m_session->previewPipeline(m_draftOps, outError);
}

bool TrajectoryEditObserver::runPreviewIfRaw(QString* outError)
{
	if (!m_session)
	{
		return false;
	}
	RobotInstruction::RawTrajectory preview{};
	if (!m_session->buildRawPreviewWithPipeline(m_draftOps, preview, outError))
	{
		return false;
	}
	(void)preview;
	return true;
}

bool TrajectoryEditObserver::apply(QString* outError)
{
	if (!m_session || !m_editService)
	{
		if (outError)
		{
			*outError = QStringLiteral("not ready");
		}
		return false;
	}
	m_session->updatePipelineOps(m_draftOps, false);
	return m_session->apply(outError);
}
