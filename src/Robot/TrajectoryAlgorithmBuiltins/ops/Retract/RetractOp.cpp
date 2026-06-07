// Retract 原子块：在路径尾端插入退刀段
#include "RetractOp.h"

#include "TrajectoryOpPathApply.h"

#include <cstdio>

namespace trajectory_algo
{
namespace
{
const char* directionLabel(const int mode, const bool chinese)
{
	switch (mode)
	{
	case static_cast<int>(RobotInstruction::ApproachDirectionMode::PathTangent):
		return chinese ? "切向" : "Tangent";
	case static_cast<int>(RobotInstruction::ApproachDirectionMode::ToolZ):
		return chinese ? "工具Z" : "ToolZ";
	case static_cast<int>(RobotInstruction::ApproachDirectionMode::SurfaceNormal):
	default:
		return chinese ? "法向" : "Normal";
	}
}

const char* segmentLabel(const int mode, const bool chinese)
{
	switch (mode)
	{
	case static_cast<int>(RobotInstruction::SegmentSelectMode::IndexRange):
		return chinese ? "区间" : "Range";
	case static_cast<int>(RobotInstruction::SegmentSelectMode::AllSegments):
	default:
		return chinese ? "全部" : "All";
	}
}

TrajectoryOpParamField boolField(
	const std::string& key,
	const std::string& labelEn,
	const std::string& labelZh,
	const bool defaultValue,
	const int order,
	const std::string& group)
{
	TrajectoryOpParamField field{};
	field.key = key;
	field.type = TrajectoryParamType::Bool;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.defaultBool = defaultValue;
	field.order = order;
	field.group = group;
	return field;
}
} // namespace

RobotInstruction::TrajectoryOpKind RetractOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Retract;
}

const char* RetractOp::displayName(const bool chinese) const
{
	return chinese ? "退刀" : "Retract";
}

TrajectoryOpCapability RetractOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor RetractOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Retract;
	op.scope = defaultScope;
	op.retract.distanceMm = 20.0;
	op.retract.directionMode = RobotInstruction::ApproachDirectionMode::SurfaceNormal;
	op.retract.insertMode = RobotInstruction::InsertMode::Trajectory;
	op.retract.segmentSelectMode = RobotInstruction::SegmentSelectMode::AllSegments;
	op.retract.segmentFrom = 1;
	op.retract.segmentTo = 1;
	op.retract.overrideSpeedEnabled = false;
	op.retract.speedMmPerSec = 100.0;
	return op;
}

std::vector<TrajectoryOpParamField> RetractOp::paramFields() const
{
	return {
		doubleParamField("retract.distanceMm", "Distance", "退刀距离", "mm", 0.0, 10000.0, 0.1, 20.0, 0, "retract"),
		enumParamField(
			"retract.directionMode",
			"Direction",
			"方向模式",
			{ "0", "1", "2" },
			{ "切向", "法向", "工具Z" },
			{ "PathTangent", "SurfaceNormal", "ToolZ" },
			1,
			1,
			"retract"),
		enumParamField(
			"retract.insertMode",
			"Insert Mode",
			"插入模式",
			{ "0", "1" },
			{ "轨迹尾", "分段尾" },
			{ "TrajectoryTail", "SegmentTail" },
			0,
			2,
			"retract"),
		enumParamField(
			"retract.segmentSelectMode",
			"Segment Scope",
			"分段范围",
			{ "0", "1" },
			{ "全部分段", "区间" },
			{ "AllSegments", "IndexRange" },
			0,
			3,
			"retract"),
		intParamField("retract.segmentFrom", "Segment From", "起始段", 1, 100000, 1, 4, "retract"),
		intParamField("retract.segmentTo", "Segment To", "结束段", 1, 100000, 1, 5, "retract"),
		boolField("retract.overrideSpeedEnabled", "Override Speed", "速度覆盖", false, 6, "retract"),
		doubleParamField("retract.speedMmPerSec", "Speed", "退刀速度", "mm/s", 1.0, 5000.0, 1.0, 100.0, 7, "retract"),
	};
}

bool RetractOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (op.retract.distanceMm < 0.0)
	{
		if (errMsg)
		{
			*errMsg = "retract distance must be >= 0";
		}
		return false;
	}
	if (op.retract.segmentFrom < 1 || op.retract.segmentTo < op.retract.segmentFrom)
	{
		if (errMsg)
		{
			*errMsg = "invalid retract segment range";
		}
		return false;
	}
	return true;
}

std::string RetractOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	char buffer[256];
	std::snprintf(
		buffer,
		sizeof(buffer),
		chinese ? "退刀 | 距离%.2f | %s | 段:%s"
				: "Retract | Dist %.2f | %s | Seg:%s",
		op.retract.distanceMm,
		directionLabel(static_cast<int>(op.retract.directionMode), chinese),
		segmentLabel(static_cast<int>(op.retract.segmentSelectMode), chinese));
	return buffer;
}

bool RetractOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	std::string* errMsg) const
{
	return applyUnifiedPathOp(op, traj, errMsg);
}

} // namespace trajectory_algo

