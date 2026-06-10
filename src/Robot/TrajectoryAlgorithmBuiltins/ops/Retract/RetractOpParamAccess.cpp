// Retract 块参数字段与 descriptor 读写
#include "RetractOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool RetractOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("retract.", 0) == 0;
}

bool RetractOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "retract.distanceMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.retract.distanceMm;
		return true;
	
	}
	if (field.key == "retract.directionMode")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.retract.directionMode);
		return true;
	
	}
	if (field.key == "retract.directionFrame")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.retract.directionFrame);
		return true;
	}
	if (field.key == "retract.customDirection.x")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.retract.customDirectionX;
		return true;
	}
	if (field.key == "retract.customDirection.y")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.retract.customDirectionY;
		return true;
	}
	if (field.key == "retract.customDirection.z")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.retract.customDirectionZ;
		return true;
	}
	if (field.key == "retract.insertMode")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.retract.insertMode);
		return true;
	
	}
	if (field.key == "retract.segmentSelectMode")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.retract.segmentSelectMode);
		return true;
	
	}
	if (field.key == "retract.segmentFrom")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.retract.segmentFrom;
		return true;
	
	}
	if (field.key == "retract.segmentTo")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = op.retract.segmentTo;
		return true;
	
	}
	if (field.key == "retract.overrideSpeedEnabled")
	{
		out.kind = TrajectoryParamValue::Kind::Bool;
		out.asBool = op.retract.overrideSpeedEnabled;
		return true;
	
	}
	if (field.key == "retract.speedMmPerSec")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.retract.speedMmPerSec;
		return true;
	
	}
	return false;
}

bool RetractOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "retract.distanceMm")
	{
		op.retract.distanceMm = in.asDouble;
		return true;
	
	}
	if (field.key == "retract.directionMode")
	{
		op.retract.directionMode = static_cast<RobotInstruction::ApproachDirectionMode>(in.asInt);
		return true;
	
	}
	if (field.key == "retract.directionFrame")
	{
		op.retract.directionFrame = static_cast<RobotInstruction::TransformReferenceFrame>(in.asInt);
		return true;
	}
	if (field.key == "retract.customDirection.x")
	{
		op.retract.customDirectionX = in.asDouble;
		return true;
	}
	if (field.key == "retract.customDirection.y")
	{
		op.retract.customDirectionY = in.asDouble;
		return true;
	}
	if (field.key == "retract.customDirection.z")
	{
		op.retract.customDirectionZ = in.asDouble;
		return true;
	}
	if (field.key == "retract.insertMode")
	{
		op.retract.insertMode = static_cast<RobotInstruction::InsertMode>(in.asInt);
		return true;
	
	}
	if (field.key == "retract.segmentSelectMode")
	{
		op.retract.segmentSelectMode = static_cast<RobotInstruction::SegmentSelectMode>(in.asInt);
		return true;
	
	}
	if (field.key == "retract.segmentFrom")
	{
		op.retract.segmentFrom = in.asInt;
		return true;
	
	}
	if (field.key == "retract.segmentTo")
	{
		op.retract.segmentTo = in.asInt;
		return true;
	
	}
	if (field.key == "retract.overrideSpeedEnabled")
	{
		op.retract.overrideSpeedEnabled = in.asBool;
		return true;
	
	}
	if (field.key == "retract.speedMmPerSec")
	{
		op.retract.speedMmPerSec = in.asDouble;
		return true;
	
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeRetractOpParamAccess()
{
	return std::make_unique<RetractOpParamAccess>();
}

} // namespace trajectory_algo
