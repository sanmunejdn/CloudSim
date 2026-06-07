// AssignSpeedZone 块参数字段与 descriptor 读写
#include "AssignSpeedZoneOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool AssignSpeedZoneOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("assign.", 0) == 0;
}

bool AssignSpeedZoneOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "assign.speedMmPerSec")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.assignMotion.speedMmPerSec;
		return true;
	}
	return false;
}

bool AssignSpeedZoneOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "assign.speedMmPerSec")
	{
		op.assignMotion.speedMmPerSec = in.asDouble;
		return true;
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeAssignSpeedZoneOpParamAccess()
{
	return std::make_unique<AssignSpeedZoneOpParamAccess>();
}

} // namespace trajectory_algo
