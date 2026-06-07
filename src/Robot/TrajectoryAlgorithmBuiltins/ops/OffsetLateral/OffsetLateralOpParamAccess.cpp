// OffsetLateral 块参数字段与 descriptor 读写
#include "OffsetLateralOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool OffsetLateralOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("offset.", 0) == 0;
}

bool OffsetLateralOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "offset.lateralMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.pathOffset.lateralMm;
		return true;
	}
	return false;
}

bool OffsetLateralOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "offset.lateralMm")
	{
		op.pathOffset.lateralMm = in.asDouble;
		return true;
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeOffsetLateralOpParamAccess()
{
	return std::make_unique<OffsetLateralOpParamAccess>();
}

} // namespace trajectory_algo
