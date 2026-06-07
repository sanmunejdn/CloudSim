// OffsetAlongNormal 块参数字段与 descriptor 读写
#include "OffsetAlongNormalOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool OffsetAlongNormalOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("offset.", 0) == 0;
}

bool OffsetAlongNormalOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "offset.offsetMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.pathOffset.offsetMm;
		return true;
	}
	return false;
}

bool OffsetAlongNormalOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "offset.offsetMm")
	{
		op.pathOffset.offsetMm = in.asDouble;
		return true;
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeOffsetAlongNormalOpParamAccess()
{
	return std::make_unique<OffsetAlongNormalOpParamAccess>();
}

} // namespace trajectory_algo
