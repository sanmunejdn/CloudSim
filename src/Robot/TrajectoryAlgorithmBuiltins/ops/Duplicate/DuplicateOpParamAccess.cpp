// Duplicate 块参数字段与 descriptor 读写
#include "DuplicateOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool DuplicateOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("structural.", 0) == 0;
}

bool DuplicateOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "structural.duplicateCount")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.duplicateCount;
		return true;
	
	}
	return false;
}

bool DuplicateOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "structural.duplicateCount")
	{
		op.duplicateCount = in.asInt;
		return true;
	
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeDuplicateOpParamAccess()
{
	return std::make_unique<DuplicateOpParamAccess>();
}

} // namespace trajectory_algo
