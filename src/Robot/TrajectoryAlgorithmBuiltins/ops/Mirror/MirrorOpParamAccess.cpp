// Mirror 块参数字段与 descriptor 读写
#include "MirrorOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool MirrorOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("mirror.", 0) == 0;
}

bool MirrorOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "mirror.axis")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.mirrorAxis;
		return true;
	
	}
	return false;
}

bool MirrorOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "mirror.axis")
	{
		op.mirrorAxis = in.asInt;
		return true;
	
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeMirrorOpParamAccess()
{
	return std::make_unique<MirrorOpParamAccess>();
}

} // namespace trajectory_algo
