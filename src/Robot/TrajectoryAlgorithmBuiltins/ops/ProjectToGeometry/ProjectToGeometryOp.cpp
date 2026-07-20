/// @file ProjectToGeometryOp.cpp
/// @brief ProjectToGeometryOp 实现

// ProjectToGeometry 原子块：沿方向投影到点云/mesh/BREP
#include "ProjectToGeometryOp.h"

#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamsParse.h"

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

RobotInstruction::TrajectoryOpDescriptor
ProjectToGeometryOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::ProjectToGeometry;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	RobotInstruction::ProjectToGeometryParams project = parseProjectParams(op.params);
	project.directionFrame = RobotInstruction::TransformReferenceFrame::World;
	project.directionX = 0.0;
	project.directionY = 0.0;
	project.directionZ = -1.0;
	project.maxDistanceMm = 5000.0;
	project.pointCloudHitRadiusMm = 2.0;
	writeProjectParams(op.params, project);

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
		messageParamField("project.targetBackendId", "Select geometry backend (point cloud / mesh / BREP).",
						  "选择几何 backend（点云 / mesh / BREP）。", 0),
		directionField,
		enumParamField("project.directionFrame", "Direction Frame", "方向参考系", {"0", "1"}, {"世界", "体"},
					   {"World", "Body"}, 0, 2, "project"),
		doubleParamField("project.maxDistanceMm", "Max Distance", "最大距离", "mm", 1.0, 100000.0, 1.0, 5000.0, 3,
						 "project"),
		doubleParamField("project.pointCloudHitRadiusMm", "Point Hit Radius", "点云命中半径", "mm", 0.01, 100.0, 0.1,
						 2.0, 4, "project"),
	};
}

bool ProjectToGeometryOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (parseProjectParams(op.params).targetBackendId.empty())
	{
		if (errMsg)
		{
			*errMsg = "project target backend is required";
		}
		return false;
	}
	const double len = std::sqrt(parseProjectParams(op.params).directionX * parseProjectParams(op.params).directionX +
								 parseProjectParams(op.params).directionY * parseProjectParams(op.params).directionY +
								 parseProjectParams(op.params).directionZ * parseProjectParams(op.params).directionZ);
	if (len < 1e-6)
	{
		if (errMsg)
		{
			*errMsg = "project direction must be non-zero";
		}
		return false;
	}
	if (parseProjectParams(op.params).maxDistanceMm <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "project max distance must be > 0";
		}
		return false;
	}
	return true;
}

std::string ProjectToGeometryOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op,
											   const bool chinese) const
{
	(void)chinese;
	return std::string("ProjectToGeometry -> ") + parseProjectParams(op.params).targetBackendId;
}

bool ProjectToGeometryOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op,
									  RobotInstruction::UnifiedTrajectory& traj,
									  const TrajectoryOpExecutionContext& ctx, std::string* errMsg) const
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
	return ctx.geometryProjection->project(traj, parseProjectParams(op.params), op.scope, ctx.program, &missCount,
										   errMsg);
}

} // namespace trajectory_algo
