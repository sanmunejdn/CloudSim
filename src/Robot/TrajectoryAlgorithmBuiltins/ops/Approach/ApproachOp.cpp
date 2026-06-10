// Approach 原子块：在路径首端插入进刀点
#include "ApproachOp.h"

#include "UnifiedTrajectorySemanticMath.h"

#include <cmath>
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
	case static_cast<int>(RobotInstruction::ApproachDirectionMode::Custom):
		return chinese ? "自定义" : "Custom";
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
		return chinese ? "范围" : "Range";
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

TrajectoryOpParamField customDirectionField()
{
	TrajectoryOpParamField field{};
	field.key = "approach.customDirection";
	field.type = TrajectoryParamType::Vec3;
	field.labelEn = "Custom Direction";
	field.labelZh = "自定义方向";
	field.minValue = -1.0;
	field.maxValue = 1.0;
	field.step = 0.01;
	field.defaultDouble = 0.0;
	field.order = 2;
	field.group = "approach";
	field.visibleWhenFieldKey = "approach.directionMode";
	field.visibleWhenIntValue = static_cast<int>(RobotInstruction::ApproachDirectionMode::Custom);
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
	op.approach.directionFrame = RobotInstruction::TransformReferenceFrame::World;
	op.approach.customDirectionX = 0.0;
	op.approach.customDirectionY = 0.0;
	op.approach.customDirectionZ = -1.0;
	op.approach.overrideSpeedEnabled = false;
	op.approach.speedMmPerSec = 100.0;
	return op;
}

std::vector<TrajectoryOpParamField> ApproachOp::paramFields() const
{
	TrajectoryOpParamField directionFrameField = enumParamField(
		"approach.directionFrame",
		"Direction Frame",
		"方向参考系",
		{ "0", "1" },
		{ "世界", "体" },
		{ "World", "Body" },
		0,
		3,
		"approach");
	directionFrameField.visibleWhenFieldKey = "approach.directionMode";
	directionFrameField.visibleWhenIntValue =
		static_cast<int>(RobotInstruction::ApproachDirectionMode::Custom);
	TrajectoryOpParamField segmentFromField = intParamField(
		"approach.segmentFrom",
		"Segment From",
		"段起始",
		1,
		100000,
		1,
		4,
		"approach");
	segmentFromField.visibleWhenFieldKey = "approach.segmentSelectMode";
	segmentFromField.visibleWhenIntValue = 1;
	TrajectoryOpParamField segmentToField = intParamField(
		"approach.segmentTo",
		"Segment To",
		"段结束",
		1,
		100000,
		1,
		5,
		"approach");
	segmentToField.visibleWhenFieldKey = "approach.segmentSelectMode";
	segmentToField.visibleWhenIntValue = 1;
	return {
		doubleParamField(
			"approach.distanceMm",
			"Distance",
			"进刀距离",
			"mm",
			0.0,
			10000.0,
			0.1,
			20.0,
			0,
			"approach"),
		enumParamField(
			"approach.directionMode",
			"Direction",
			"方向模式",
			{ "0", "1", "2", "3" },
			{ "切向", "法向", "工具Z", "自定义" },
			{ "PathTangent", "SurfaceNormal", "ToolZ", "Custom" },
			1,
			1,
			"approach"),
		customDirectionField(),
		directionFrameField,
		enumParamField(
			"approach.insertMode",
			"Insert Mode",
			"插入模式",
			{ "0", "1" },
			{ "轨迹首", "段首" },
			{ "TrajectoryHead", "SegmentHead" },
			0,
			2,
			"approach"),
		enumParamField(
			"approach.segmentSelectMode",
			"Segment Scope",
			"段选择",
			{ "0", "1" },
			{ "全部段", "范围" },
			{ "AllSegments", "IndexRange" },
			0,
			3,
			"approach"),
		segmentFromField,
		segmentToField,
		boolField("approach.overrideSpeedEnabled", "Override Speed", "覆盖速度", false, 6, "approach"),
		doubleParamField(
			"approach.speedMmPerSec",
			"Speed",
			"速度",
			"mm/s",
			1.0,
			5000.0,
			1.0,
			100.0,
			7,
			"approach"),
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
	if (op.approach.directionMode == RobotInstruction::ApproachDirectionMode::Custom)
	{
		const double len = std::sqrt(
			op.approach.customDirectionX * op.approach.customDirectionX
			+ op.approach.customDirectionY * op.approach.customDirectionY
			+ op.approach.customDirectionZ * op.approach.customDirectionZ);
		if (len < 1e-6)
		{
			if (errMsg)
			{
				*errMsg = "approach custom direction must be non-zero";
			}
			return false;
		}
	}
	return true;
}

std::string ApproachOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
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

bool ApproachOp::processPath(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	RobotInstruction::UnifiedTrajectory& traj,
	const TrajectoryOpExecutionContext& ctx,
	std::string* errMsg) const
{
	(void)errMsg;
	insertApproachInScope(traj, op.approach, op.scope, ctx.program);
	return true;
}

} // namespace trajectory_algo
