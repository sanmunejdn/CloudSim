#include "ApproachOp.h"

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

RobotInstruction::TrajectoryOpKind ApproachOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Approach;
}

const char* ApproachOp::displayName(const bool chinese) const
{
	return chinese ? "进刀" : "Approach";
}

TrajectoryOpCapability ApproachOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor ApproachOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Approach;
	op.scope = defaultScope;
	op.approach.distanceMm = 20.0;
	op.approach.directionMode = RobotInstruction::ApproachDirectionMode::SurfaceNormal;
	op.approach.insertMode = RobotInstruction::InsertMode::Trajectory;
	op.approach.segmentSelectMode = RobotInstruction::SegmentSelectMode::AllSegments;
	op.approach.segmentFrom = 1;
	op.approach.segmentTo = 1;
	op.approach.overrideSpeedEnabled = false;
	op.approach.speedMmPerSec = 100.0;
	return op;
}

std::vector<TrajectoryOpParamField> ApproachOp::paramFields() const
{
	return {
		doubleParamField("approach.distanceMm", "Distance", "进刀距离", "mm", 0.0, 10000.0, 0.1, 20.0, 0, "approach"),
		enumParamField(
			"approach.directionMode",
			"Direction",
			"方向模式",
			{ "0", "1", "2" },
			{ "切向", "法向", "工具Z" },
			{ "PathTangent", "SurfaceNormal", "ToolZ" },
			1,
			1,
			"approach"),
		enumParamField(
			"approach.insertMode",
			"Insert Mode",
			"插入模式",
			{ "0", "1" },
			{ "轨迹头", "分段头" },
			{ "TrajectoryHead", "SegmentHead" },
			0,
			2,
			"approach"),
		enumParamField(
			"approach.segmentSelectMode",
			"Segment Scope",
			"分段范围",
			{ "0", "1" },
			{ "全部分段", "区间" },
			{ "AllSegments", "IndexRange" },
			0,
			3,
			"approach"),
		intParamField("approach.segmentFrom", "Segment From", "起始段", 1, 100000, 1, 4, "approach"),
		intParamField("approach.segmentTo", "Segment To", "结束段", 1, 100000, 1, 5, "approach"),
		boolField("approach.overrideSpeedEnabled", "Override Speed", "速度覆盖", false, 6, "approach"),
		doubleParamField("approach.speedMmPerSec", "Speed", "进刀速度", "mm/s", 1.0, 5000.0, 1.0, 100.0, 7, "approach"),
	};
}

bool ApproachOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (op.approach.distanceMm < 0.0)
	{
		if (errMsg)
		{
			*errMsg = "approach distance must be >= 0";
		}
		return false;
	}
	if (op.approach.segmentFrom < 1 || op.approach.segmentTo < op.approach.segmentFrom)
	{
		if (errMsg)
		{
			*errMsg = "invalid approach segment range";
		}
		return false;
	}
	return true;
}

std::string ApproachOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	char buffer[256];
	std::snprintf(
		buffer,
		sizeof(buffer),
		chinese ? "进刀 | 距离%.2f | %s | 段:%s"
				: "Approach | Dist %.2f | %s | Seg:%s",
		op.approach.distanceMm,
		directionLabel(static_cast<int>(op.approach.directionMode), chinese),
		segmentLabel(static_cast<int>(op.approach.segmentSelectMode), chinese));
	return buffer;
}

bool ApproachOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	(void)op;
	(void)targetIds;
	(void)out;
	return false;
}

std::vector<TrajectoryApplyAction> ApproachOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	(void)op;
	return {};
}

} // namespace trajectory_algo

