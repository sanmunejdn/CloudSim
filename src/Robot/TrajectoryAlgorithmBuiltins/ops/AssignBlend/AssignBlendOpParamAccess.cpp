// AssignBlend 块参数字段与 descriptor 读写
#include "AssignBlendOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool AssignBlendOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("assign.", 0) == 0;
}

bool AssignBlendOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "assign.blendRadiusMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.assignMotion.blendRadiusMm;
		return true;
	}
	return false;
}

bool AssignBlendOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "assign.blendRadiusMm")
	{
		op.assignMotion.blendRadiusMm = in.asDouble;
		return true;
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeAssignBlendOpParamAccess()
{
	return std::make_unique<AssignBlendOpParamAccess>();
}

} // namespace trajectory_algo
