#include "KinematicModelApply.h"

#include "CompositeKinematicModel.h"
#include "CustomDeviceKinematicModel.h"
#include "CustomDeviceKinematics.h"
#include "KinematicModelRegistry.h"
#include "UrdfRobotKinematicModel.h"
#include "UrdfRobotKinematicModelSink.h"

#include "BackendDataManager.h"
#include "IRobotBackendPoseSink.h"
#include "IRobotSimulationDocument.h"

namespace KinematicModelApply
{
bool applyCustomDevice(const std::string& registryKey, CustomDeviceBackendData& device, BackendDataManager* mgr,
					   IRobotBackendPoseSink* sink, const std::vector<double>& q)
{
	std::shared_ptr<kinematic_core::IKinematicModel> base = KinematicModelRegistry::modelForKey(registryKey);
	auto model = std::dynamic_pointer_cast<CustomDeviceKinematicModel::Model>(base);
	if (!model)
	{
		model = CustomDeviceKinematicModel::create(device);
		KinematicModelRegistry::registerModel(registryKey, model);
	}
	const BackendMat4 w0Mat = CustomDeviceKinematics::resolveEffectiveDeviceW0(device, mgr);
	double w0[16];
	for (int i = 0; i < 16; ++i)
	{
		w0[i] = w0Mat.v[i];
	}
	model->rebuildGraph();
	return model->applyToSink(device, mgr, sink, q, w0);
}

bool applyRobotArm(const std::string& registryKey, const RobotKinematicApplyContext::Context& ctx,
				   const std::vector<double>& localArmQ, QVector<double>& aggregatedAnglesRad)
{
	if (!ctx.doc || !ctx.sink || ctx.instanceIndex < 0)
	{
		return false;
	}
	std::shared_ptr<kinematic_core::IKinematicModel> base = KinematicModelRegistry::modelForKey(registryKey);
	if (!base)
	{
		const QString urdfPath = ctx.doc->robotUrdfAbsolutePathForInstance(ctx.instanceIndex);
		if (urdfPath.isEmpty())
		{
			return false;
		}
		base = UrdfRobotKinematicModel::create(urdfPath);
		KinematicModelRegistry::registerModel(registryKey, base);
	}
	if (auto composite = std::dynamic_pointer_cast<CompositeKinematicModel::Model>(base))
	{
		const std::vector<double> extQ = ctx.doc->robotExternalAxisQ(ctx.instanceIndex);
		return composite->applyToSink(ctx, localArmQ, extQ.empty() ? nullptr : &extQ, aggregatedAnglesRad);
	}
	if (auto arm = std::dynamic_pointer_cast<UrdfRobotKinematicModel::Model>(base))
	{
		return UrdfRobotKinematicModelSink::applyToSink(*arm, ctx, localArmQ, aggregatedAnglesRad);
	}
	return false;
}

} // namespace KinematicModelApply
