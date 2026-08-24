#include "RobotKinematicModelRegistration.h"

#include "CompositeKinematicModel.h"
#include "ExternalAxisKinematicModel.h"
#include "IRobotSimulationDocument.h"
#include "RobotExternalAxes.h"
#include "KinematicModelRegistry.h"
#include "UrdfRobotKinematicModel.h"

namespace RobotKinematicModelRegistration
{
bool registerRobotInstance(IRobotSimulationDocument* doc, const int instanceIndex, const QString& sceneRootBackendId)
{
	if (!doc || instanceIndex < 0 || sceneRootBackendId.isEmpty())
	{
		return false;
	}
	const QString urdfPath = doc->robotUrdfAbsolutePathForInstance(instanceIndex);
	if (urdfPath.isEmpty())
	{
		return false;
	}
	auto composite = std::make_shared<CompositeKinematicModel::Model>();
	composite->addSegment(UrdfRobotKinematicModel::create(urdfPath));
	const RobotExternal::RobotExternalAxisConfigSet extAxes = doc->robotExternalAxesForInstance(instanceIndex);
	const auto extModel =
		ExternalAxisKinematicModel::create(extAxes, RobotExternal::RobotExternalAttachment::RobotBase);
	if (extModel && extModel->dofCount() > 0)
	{
		composite->addSegment(extModel);
	}
	KinematicModelRegistry::registerModel(KinematicModelRegistry::keyRobotInstance(sceneRootBackendId.toStdString()),
										  composite);
	return true;
}

} // namespace RobotKinematicModelRegistration
