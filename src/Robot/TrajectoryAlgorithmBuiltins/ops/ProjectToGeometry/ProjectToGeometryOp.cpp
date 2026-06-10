// ProjectToGeometry 原子块：沿方向投影到点云/mesh/BREP
#include "ProjectToGeometryOp.h"

#include <cmath>

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind ProjectToGeometryOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::ProjectToGeometry;
}

const char* ProjectToGeometryOp::displayName(const bool chinese) const
{
	return chinese ? "轨迹投影" : "ProjectToGeometry";
}

TrajectoryOpCapability ProjectToGeometryOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor ProjectToGeometryOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ProjectToGeometry;
	op.scope = defaultScope;
	op.project.directionFrame = RobotInstruction::TransformReferenceFrame::World;
	op.project.directionX = 0.0;
	op.project.directionY = 0.0;
	op.project.directionZ = -1.0;
	op.project.maxDistanceMm = 5000.0;
	op.project.pointCloudHitRadiusMm = 2.0;
	return op;
}

std::vector<TrajectoryOpParamField> ProjectToGeometryOp::paramFields() const
{
	TrajectoryOpParamField directionField{};
	directionField.key = "project.direction";
	directionField.type = TrajectoryParamType::Vec3;
	directionField.labelEn = "Direction";
	directionField.labelZh = "投影方向";
	directionField.minValue = -1.0;
	directionField.maxValue = 1.0;
	directionField.step = 0.01;
	directionField.order = 1;
	directionField.group = "project";
	return {
		messageParamField(
			"project.targetBackendId",
			"Select geometry backend (point cloud / mesh / BREP).",
			"选择几何 backend（点云 / mesh / BREP）。",
			0),
		directionField,
		enumParamField(
			"project.directionFrame",
			"Direction Frame",
			"方向参考系",
			{ "0", "1" },
			{ "世界", "体" },
			{ "World", "Body" },
			0,
			2,
			"project"),
		doubleParamField(
			"project.maxDistanceMm",
			"Max Distance",
			"最大距离",
			"mm",
			1.0,
			100000.0,
			1.0,
			5000.0,
			3,
			"project"),
		doubleParamField(
			"project.pointCloudHitRadiusMm",
			"Point Hit Radius",
			"点云命中半径",
			"mm",
			0.01,
			100.0,
			0.1,
			2.0,
			4,
			"project"),
	};
}

bool ProjectToGeometryOp::validate(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	std::string* errMsg) const
{
	if (op.project.targetBackendId.empty())
	{
		if (errMsg)
		{
			*errMsg = "project target backend is required";
		}
		return false;
	}
	const double len = std::sqrt(
		op.project.directionX * op.project.directionX
		+ op.project.directionY * op.project.directionY
		+ op.project.directionZ * op.project.directionZ);
	if (len < 1e-6)
	{
		if (errMsg)
		{
			*errMsg = "project direction must be non-zero";
		}
		return false;
	}
	if (op.project.maxDistanceMm <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "project max distance must be > 0";
		}
		return false;
	}
	return true;
}

std::string ProjectToGeometryOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)chinese;
	return std::string("ProjectToGeometry -> ") + op.project.targetBackendId;
}

bool ProjectToGeometryOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	const TrajectoryOpExecutionContext& ctx,
	std::string* errMsg) const
{
	if (!ctx.geometryProjection)
	{
		if (errMsg)
		{
			*errMsg = "geometry projection not available";
		}
		return false;
	}
	std::size_t missCount = 0;
	return ctx.geometryProjection->project(
		traj,
		op.project,
		op.scope,
		ctx.program,
		&missCount,
		errMsg);
}

} // namespace trajectory_algo
