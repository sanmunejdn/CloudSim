/// @file WeaveOp.cpp
/// @brief Weave 轨迹算子

// Weave：物体系沿工具 Y 位置摆，可选绕工具轴姿态摆
#include "WeaveOp.h"

#include "TrajectoryOpFormat.h"
#include "TrajectoryOpParamAccess.h"
#include "TrajectoryOpParamSchema.h"
#include "TrajectoryOpParamsParse.h"
#include "UnifiedTrajectoryPathMath.h"

#include <cstdio>

namespace trajectory_algo
{
namespace
{
TrajectoryOpParamField boolParamField(const std::string& key, const std::string& labelEn, const std::string& labelZh,
									  const bool defaultValue, const int order)
{
	TrajectoryOpParamField field{};
	field.key = key;
	field.type = TrajectoryParamType::Bool;
	field.labelEn = labelEn;
	field.labelZh = labelZh;
	field.defaultBool = defaultValue;
	field.order = order;
	return field;
}

const char* poseAxisLabel(const int axis, const bool chinese)
{
	switch (axis)
	{
	case 1:
		return chinese ? "绕Y" : "about Y";
	case 2:
		return chinese ? "绕Z" : "about Z";
	case 0:
	default:
		return chinese ? "绕X" : "about X";
	}
}
} // namespace

RobotInstruction::TrajectoryOpKind WeaveOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Weave;
}

const char* WeaveOp::displayName(const bool chinese) const
{
	return chinese ? "摆动" : "Weave";
}

TrajectoryOpCapability WeaveOp::capabilities() const
{
	return TrajectoryOpCapability::None;
}

RobotInstruction::TrajectoryOpDescriptor
WeaveOp::makeDefaultDescriptor(const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Weave;
	op.scope = defaultScope;
	TrajectoryOpParamAccess::applyDefaults(op, *this);
	writeWeaveParams(op.params, RobotInstruction::WeaveParams{});

	return op;
}

std::vector<TrajectoryOpParamField> WeaveOp::paramFields() const
{
	return {
		doubleParamField("weave.amplitudeMm", "Position amplitude", "位置振幅", "mm", -1e5, 1e5, 0.01, 2.0, 0),
		doubleParamField("weave.periodMm", "Period", "周期", "mm", 0.01, 1e5, 0.01, 10.0, 1),
		boolParamField("weave.wavePose", "Wave pose", "姿态摆", false, 2),
		doubleParamField("weave.poseAmplitudeDeg", "Pose amplitude", "姿态振幅", "deg", -180.0, 180.0, 0.1, 5.0, 3),
		enumParamField("weave.poseAxis", "Pose axis", "姿态摆轴", {"0", "1", "2"}, {"绕工具X", "绕工具Y", "绕工具Z"},
					   {"About X", "About Y", "About Z"}, 0, 4),
	};
}

bool WeaveOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	const RobotInstruction::WeaveParams p = parseWeaveParams(op.params);
	if (p.periodMm <= 0.0)
	{
		if (errMsg)
		{
			*errMsg = "weave period must be > 0";
		}
		return false;
	}
	if (p.poseAxis < 0 || p.poseAxis > 2)
	{
		if (errMsg)
		{
			*errMsg = "weave poseAxis must be 0..2";
		}
		return false;
	}
	return true;
}

std::string WeaveOp::formatSummary(const RobotInstruction::TrajectoryOpDescriptor& op, const bool chinese) const
{
	const RobotInstruction::WeaveParams p = parseWeaveParams(op.params);
	char buffer[160];
	if (p.wavePose)
	{
		std::snprintf(buffer, sizeof(buffer),
					  chinese ? "摆动 | BodyY %.1fmm 周期%.1f | 姿态%s %.1fdeg" : "Weave | BodyY %.1fmm T=%.1f | pose %s %.1fdeg",
					  p.amplitudeMm, p.periodMm, poseAxisLabel(p.poseAxis, chinese), p.poseAmplitudeDeg);
	}
	else
	{
		std::snprintf(buffer, sizeof(buffer), chinese ? "摆动 | BodyY %.1fmm 周期%.1f" : "Weave | BodyY %.1fmm T=%.1f",
					  p.amplitudeMm, p.periodMm);
	}
	return buffer;
}

bool WeaveOp::processPath(const RobotInstruction::TrajectoryOpDescriptor& op, RobotInstruction::UnifiedTrajectory& traj,
						  const TrajectoryOpExecutionContext& ctx, std::string* errMsg) const
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	weaveUnifiedInScope(traj, op.scope, ctx.program, parseWeaveParams(op.params));
	return true;
}

} // namespace trajectory_algo
