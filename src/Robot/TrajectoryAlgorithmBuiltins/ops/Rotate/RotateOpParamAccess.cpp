// Rotate 块参数字段与 descriptor 读写
#include "RotateOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool RotateOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("rotate.", 0) == 0;
}

bool RotateOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "rotate.frame")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.rotate.frame);
		return true;
	
	}
	if (field.key == "rotate.axisX")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.axisX;
		return true;
	
	}
	if (field.key == "rotate.axisY")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.axisY;
		return true;
	
	}
	if (field.key == "rotate.axisZ")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.axisZ;
		return true;
	
	}
	if (field.key == "rotate.angleDeg")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.angleDeg;
		return true;
	
	}
	if (field.key == "rotate.endAngleDeg")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.rotate.endAngleDeg;
		return true;
	
	}
	return false;
}

bool RotateOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "rotate.frame")
	{
		op.rotate.frame = static_cast<RobotInstruction::TransformReferenceFrame>(in.asInt);
		return true;
	
	}
	if (field.key == "rotate.axisX")
	{
		op.rotate.axisX = in.asDouble;
		return true;
	
	}
	if (field.key == "rotate.axisY")
	{
		op.rotate.axisY = in.asDouble;
		return true;
	
	}
	if (field.key == "rotate.axisZ")
	{
		op.rotate.axisZ = in.asDouble;
		return true;
	
	}
	if (field.key == "rotate.angleDeg")
	{
		op.rotate.angleDeg = in.asDouble;
		return true;
	
	}
	if (field.key == "rotate.endAngleDeg")
	{
		op.rotate.endAngleDeg = in.asDouble;
		return true;
	
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeRotateOpParamAccess()
{
	return std::make_unique<RotateOpParamAccess>();
}

} // namespace trajectory_algo
