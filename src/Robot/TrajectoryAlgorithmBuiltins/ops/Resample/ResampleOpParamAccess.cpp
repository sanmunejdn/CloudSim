// Resample 块参数字段与 descriptor 读写
#include "ResampleOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool ResampleOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("resample.", 0) == 0;
}

bool ResampleOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "resample.stepMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.resample.stepMm;
		return true;
	}
	return false;
}

bool ResampleOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "resample.stepMm")
	{
		op.resample.stepMm = in.asDouble;
		return true;
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeResampleOpParamAccess()
{
	return std::make_unique<ResampleOpParamAccess>();
}

} // namespace trajectory_algo
