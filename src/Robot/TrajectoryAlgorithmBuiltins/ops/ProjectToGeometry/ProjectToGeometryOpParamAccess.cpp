// ProjectToGeometry 块参数字段与 descriptor 读写
#include "ProjectToGeometryOpParamAccess.h"

#include "TrajectoryPipelineTypes.h"

namespace trajectory_algo
{

bool ProjectToGeometryOpParamAccess::handlesKey(const std::string& key) const
{
	return key.rfind("project.", 0) == 0;
}

bool ProjectToGeometryOpParamAccess::read(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	TrajectoryParamValue& out) const
{
	if (field.key == "project.targetBackendId")
	{
		out.kind = TrajectoryParamValue::Kind::String;
		out.asString = op.project.targetBackendId;
		return true;
	}
	if (field.key == "project.directionFrame")
	{
		out.kind = TrajectoryParamValue::Kind::Int;
		out.asInt = static_cast<int>(op.project.directionFrame);
		return true;
	}
	if (field.key == "project.direction.x")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.project.directionX;
		return true;
	}
	if (field.key == "project.direction.y")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.project.directionY;
		return true;
	}
	if (field.key == "project.direction.z")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.project.directionZ;
		return true;
	}
	if (field.key == "project.maxDistanceMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.project.maxDistanceMm;
		return true;
	}
	if (field.key == "project.pointCloudHitRadiusMm")
	{
		out.kind = TrajectoryParamValue::Kind::Double;
		out.asDouble = op.project.pointCloudHitRadiusMm;
		return true;
	}
	return false;
}

bool ProjectToGeometryOpParamAccess::write(
	RobotInstruction::TrajectoryOpDescriptor& op,
	const TrajectoryOpParamField& field,
	const TrajectoryParamValue& in) const
{
	if (field.key == "project.targetBackendId")
	{
		op.project.targetBackendId = in.asString;
		return true;
	}
	if (field.key == "project.directionFrame")
	{
		op.project.directionFrame = static_cast<RobotInstruction::TransformReferenceFrame>(in.asInt);
		return true;
	}
	if (field.key == "project.direction.x")
	{
		op.project.directionX = in.asDouble;
		return true;
	}
	if (field.key == "project.direction.y")
	{
		op.project.directionY = in.asDouble;
		return true;
	}
	if (field.key == "project.direction.z")
	{
		op.project.directionZ = in.asDouble;
		return true;
	}
	if (field.key == "project.maxDistanceMm")
	{
		op.project.maxDistanceMm = in.asDouble;
		return true;
	}
	if (field.key == "project.pointCloudHitRadiusMm")
	{
		op.project.pointCloudHitRadiusMm = in.asDouble;
		return true;
	}
	return false;
}

std::unique_ptr<IOpParamAccess> makeProjectToGeometryOpParamAccess()
{
	return std::make_unique<ProjectToGeometryOpParamAccess>();
}

} // namespace trajectory_algo
