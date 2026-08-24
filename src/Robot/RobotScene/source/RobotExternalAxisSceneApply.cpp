#include "RobotExternalAxisSceneApply.h"

#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"
#include "RobotExternalAxes.h"

#include <CoreTypes.h>

#include <QSet>
#include <QString>

namespace RobotExternalAxisSceneApply
{
bool applyExternalAxisQ(IRobotSimulationDocument* doc, IRobotBackendPoseSink* sink, const int instanceIndex,
						const std::vector<double>& fullQ)
{
	if (!doc || instanceIndex < 0)
	{
		return false;
	}
	const RobotExternal::RobotExternalAxisConfigSet& set = doc->robotExternalAxesForInstance(instanceIndex);
	const std::vector<double> prevQ = doc->robotExternalAxisQ(instanceIndex);

	QSet<QString> workpieceBackends;
	const std::vector<int> idxs = RobotExternal::enabledExternalAxisIndices(set);
	for (const int idx : idxs)
	{
		if (idx < 0 || idx >= static_cast<int>(set.axes.size()))
		{
			continue;
		}
		const RobotExternal::RobotExternalAxisConfig& a = set.axes[static_cast<size_t>(idx)];
		if (a.attachment != RobotExternal::RobotExternalAttachment::Workpiece || a.boundBackendId.empty())
		{
			continue;
		}
		workpieceBackends.insert(QString::fromStdString(a.boundBackendId));
	}

	if (sink)
	{
		for (const QString& backendId : workpieceBackends)
		{
			cloudsim::core::Mat4 currentWorld = cloudsim::core::PlanContextDto::identityMat4();
			if (!sink->getBackendRootWorldMatrix(backendId.toStdString(), currentWorld))
			{
				continue;
			}
			cloudsim::core::Mat4 w0Candidate = currentWorld;
			RobotExternal::unbakeWorkpiecePlacementExternalAxis(currentWorld.data(), set, backendId.toStdString(),
																prevQ, w0Candidate.data());
			doc->ensureWorkpieceExternalBasePlacement(instanceIndex, backendId, w0Candidate);
		}
	}

	doc->setRobotExternalAxisQ(instanceIndex, fullQ);

	if (sink)
	{
		for (const QString& backendId : workpieceBackends)
		{
			const std::string bid = backendId.toStdString();
			const cloudsim::core::Mat4 w0 = doc->workpieceExternalBasePlacement(instanceIndex, backendId);
			cloudsim::core::Mat4 wEff = cloudsim::core::PlanContextDto::identityMat4();
			RobotExternal::composeWorkpiecePlacementWithExternalAxis(w0.data(), set, bid, fullQ, wEff.data());
			sink->setBackendRootWorldMatrixFromWorld(bid, wEff);
		}
	}
	return true;
}

} // namespace RobotExternalAxisSceneApply
