#include "UrdfRobotKinematicModelSink.h"

#include "BackendDataManager.h"
#include "IRobotSimulationDocument.h"
#include "RobotPerLinkKinematicsSliceOsg.h"
#include "RobotSceneKinematics.h"

namespace UrdfRobotKinematicModelSink
{
bool applyToSink(const UrdfRobotKinematicModel::Model& model, const RobotKinematicApplyContext::Context& ctx,
				 const std::vector<double>& localArmQ, QVector<double>& aggregatedAnglesRad)
{
	(void)model;	if (!ctx.doc || !ctx.sink || ctx.instanceIndex < 0)
	{
		return false;
	}
	const int nj = ctx.doc->robotRevoluteJointCountForInstance(ctx.instanceIndex);
	if (static_cast<int>(localArmQ.size()) != nj)
	{
		return false;
	}

	const int total = ctx.doc->robotRevoluteJointNames().size();
	if (aggregatedAnglesRad.size() != total)
	{
		const int oldSize = aggregatedAnglesRad.size();
		aggregatedAnglesRad.resize(total);
		for (int i = oldSize; i < total; ++i)
		{
			aggregatedAnglesRad[i] = 0.0;
		}
	}
	int offset = 0;
	for (int i = 0; i < ctx.instanceIndex; ++i)
	{
		offset += ctx.doc->robotRevoluteJointCountForInstance(i);
	}
	QVector<double> local(static_cast<int>(localArmQ.size()));
	for (int i = 0; i < nj; ++i)
	{
		local[i] = localArmQ[static_cast<size_t>(i)];
		aggregatedAnglesRad[offset + i] = local[i];
	}

	cloudsim::core::RobotPerLinkKinematicsSliceDto plDto;
	if (!ctx.doc->robotPerLinkKinematicsForInstance(ctx.instanceIndex, plDto))
	{
		return RobotSceneKinematics::applyJointAnglesForInstance(ctx.doc, ctx.sink, ctx.instanceIndex, local,
																 aggregatedAnglesRad);
	}
	BackendDataManager* mgr = ctx.doc->robotBackendManagerForKinematics();
	if (!mgr)
	{
		return false;
	}
	const RobotPerLinkKinematicsSlice slice = RobotSceneKinematics::robotPerLinkSliceFromDto(plDto);

	if (!RobotSceneKinematics::applyJointAnglesViaLinkBackends(ctx.doc, ctx.sink, *mgr, local, slice))
	{
		return false;
	}
	ctx.doc->noteRobotJointAnglesAppliedForInstance(ctx.instanceIndex, local);
	ctx.doc->notifyRobotKinematicsAppliedToScene();
	return true;
}

} // namespace UrdfRobotKinematicModelSink
