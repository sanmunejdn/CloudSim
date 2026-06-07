// Approach 块参数字段与 descriptor 读写
#include "ApproachOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool ApproachOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("approach.", 0) == 0;
}

bool ApproachOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "approach.distanceMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.approach.distanceMm;
		return true;
	
	}
	if (field.key == "approach.directionMode")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.approach.directionMode);
		return true;
	
	}
	if (field.key == "approach.insertMode")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.approach.insertMode);
		return true;
	
	}
	if (field.key == "approach.segmentSelectMode")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.approach.segmentSelectMode);
		return true;
	
	}
	if (field.key == "approach.segmentFrom")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.approach.segmentFrom;
		return true;
	
	}
	if (field.key == "approach.segmentTo")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.approach.segmentTo;
		return true;
	
	}
	if (field.key == "approach.overrideSpeedEnabled")
	{
		out.kind = TrajectoryParamValue::Kind::Bool;
		out.asBool = op.approach.overrideSpeedEnabled;
		return true;
	
	}
	if (field.key == "approach.speedMmPerSec")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.approach.speedMmPerSec;
		return true;
	
	}
	return false;
}

bool ApproachOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "approach.distanceMm")
	{
		op.approach.distanceMm = in.asDouble;
		return true;
	
	}
	if (field.key == "approach.directionMode")
	{
		op.approach.directionMode = static_cast<RobotInstruction::ApproachDirectionMode>(in.asInt);
		return true;
	
	}
	if (field.key == "approach.insertMode")
	{
		op.approach.insertMode = static_cast<RobotInstruction::InsertMode>(in.asInt);
		return true;
	
	}
	if (field.key == "approach.segmentSelectMode")
	{
		op.approach.segmentSelectMode = static_cast<RobotInstruction::SegmentSelectMode>(in.asInt);
		return true;
	
	}
	if (field.key == "approach.segmentFrom")
	{
		op.approach.segmentFrom = in.asInt;
		return true;
	
	}
	if (field.key == "approach.segmentTo")
	{
		op.approach.segmentTo = in.asInt;
		return true;
	
	}
	if (field.key == "approach.overrideSpeedEnabled")
	{
		op.approach.overrideSpeedEnabled = in.asBool;
		return true;
	
	}
	if (field.key == "approach.speedMmPerSec")
	{
		op.approach.speedMmPerSec = in.asDouble;
		return true;
	
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeApproachOpParamAccess()
{
	return std::make_unique<ApproachOpParamAccess>();
}

} // namespace trajectory_algo
