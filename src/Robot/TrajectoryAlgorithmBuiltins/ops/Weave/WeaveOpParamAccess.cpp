// Weave 块参数字段与 descriptor 读写
#include "WeaveOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool WeaveOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("weave.", 0) == 0;
}

bool WeaveOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "weave.amplitudeMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.weave.amplitudeMm;
		return true;
	}
	if (field.key == "weave.periodMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.weave.periodMm;
		return true;
	}
	return false;
}

bool WeaveOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "weave.amplitudeMm")
	{
		op.weave.amplitudeMm = in.asDouble;
		return true;
	}
	if (field.key == "weave.periodMm")
	{
		op.weave.periodMm = in.asDouble;
		return true;
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeWeaveOpParamAccess()
{
	return std::make_unique<WeaveOpParamAccess>();
}

} // namespace trajectory_algo
