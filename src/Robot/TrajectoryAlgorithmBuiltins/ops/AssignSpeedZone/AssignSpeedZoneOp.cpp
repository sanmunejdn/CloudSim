// AssignSpeedZone 原子块：为路径点写入速度区
#include "AssignSpeedZoneOp.h"

#include "TrajectoryOpFormat.h"
#include "UnifiedTrajectoryPathMath.h"

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind AssignSpeedZoneOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::AssignSpeedZone;
}

const char* AssignSpeedZoneOp::displayName(const bool chinese) const
{
	return chinese ? "速度区" : "AssignSpeedZone";
}

TrajectoryOpCapability AssignSpeedZoneOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor AssignSpeedZoneOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::AssignSpeedZone;
	op.scope = defaultScope;
	return op;
}

std::vector<TrajectoryOpParamField> AssignSpeedZoneOp::paramFields() const
{
	return {
		doubleParamField("assign.speedMmPerSec", "Speed", "速度", "mm/s", -1e5, 1e5, 0.01, 100.0, 0),
	};
}

bool AssignSpeedZoneOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string AssignSpeedZoneOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	(void)op;
	return displayName(chinese);
}

bool AssignSpeedZoneOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	assignSpeedUnifiedInScope(
		traj,
		op.scope,
		activeProgramContext(),
		op.assignMotion.speedMmPerSec);
	return true;
}

} // namespace trajectory_algo
