#include "UrdfRobotKinematicModelSink.h"

#include "BackendDataManager.h"
#include "IRobotSimulationDocument.h"
#include "RobotPerLinkKinematicsApply.h"
#include "RobotPerLinkKinematicsSliceOsg.h"
#include "RobotSceneKinematics.h"
#include "UrdfRobotLoader.h"

namespace UrdfRobotKinematicModelSink
{
bool applyToSink(const UrdfRobotKinematicModel::Model& model, const RobotKinematicApplyContext::Context& ctx,
				 const std::vector<double>& localArmQ, QVector<double>& aggregatedAnglesRad)
{
	if (!ctx.doc || !ctx.sink || ctx.instanceIndex < 0)
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
		aggregatedAnglesRad.resize(total);
		aggregatedAnglesRad.fill(0.0);
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

	std::vector<std::array<double, 16>> linkWorld;
	if (!model.forward(localArmQ.data(), static_cast<std::size_t>(localArmQ.size()), linkWorld))
	{
		return false;
	}
	QHash<QString, osg::Matrixd> meshWorldTq;
	QString fkErr;
	if (!UrdfRobotLoader::computeMeshWorldFromCoreLinkWorld(slice.urdfAbsolutePath, model.graph(), linkWorld,
															meshWorldTq, slice.meshVerticesInLinkFrame, &fkErr))
	{
		return false;
	}
	if (!RobotPerLinkKinematicsApply::applyLinkWorldFromCoreFk(ctx.sink, *mgr, slice, meshWorldTq))
	{
		return false;
	}
	ctx.doc->notifyRobotKinematicsAppliedToScene();
	return true;
}

} // namespace UrdfRobotKinematicModelSink
