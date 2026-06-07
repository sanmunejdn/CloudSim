// Mirror 原子块：镜像 scope 内路点
#include "MirrorOp.h"

#include "TrajectoryOpPathApply.h"

#include <cstdio>

namespace trajectory_algo
{
namespace
{
bool validMirrorAxis(const int axis)
{
	return axis >= 0 && axis <= 2;
}

const char* axisLabel(const int axis, const bool chinese)
{
	switch (axis)
	{
	case 0:
		return chinese ? "X 轴反向" : "Reverse X";
	case 1:
		return chinese ? "Y 轴反向" : "Reverse Y";
	case 2:
	default:
		return chinese ? "Z 轴反向" : "Reverse Z";
	}
}
} // namespace

RobotInstruction::TrajectoryOpKind MirrorOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Mirror;
}

const char* MirrorOp::displayName(const bool chinese) const
{
	return chinese ? "轴反向" : "Axis Reverse";
}

TrajectoryOpCapability MirrorOp::capabilities() const
{
	return TrajectoryOpCapability::PreviewPoseTransform;
}

RobotInstruction::TrajectoryOpDescriptor MirrorOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Mirror;
	op.scope = defaultScope;
	op.mirrorAxis = 0;
	return op;
}

std::vector<TrajectoryOpParamField> MirrorOp::paramFields() const
{
	return {
		enumParamField(
			"mirror.axis",
			"Reverse Axis",
			"反向轴",
			{ "0", "1", "2" },
			{ "X 轴", "Y 轴", "Z 轴" },
			{ "Axis X", "Axis Y", "Axis Z" },
			0,
			0,
			"transform"),
	};
}

bool MirrorOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (!validMirrorAxis(op.mirrorAxis))
	{
		if (errMsg)
		{
			*errMsg = "mirror axis must be 0(X), 1(Y), or 2(Z)";
		}
		return false;
	}
	return true;
}

std::string MirrorOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	char buffer[128];
	std::snprintf(
		buffer,
		sizeof(buffer),
		chinese ? "轴反向 | %s" : "Axis Reverse | %s",
		axisLabel(op.mirrorAxis, chinese));
	return buffer;
}

bool MirrorOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	std::string* errMsg) const
{
	return applyUnifiedPathOp(op, traj, errMsg);
}

} // namespace trajectory_algo
