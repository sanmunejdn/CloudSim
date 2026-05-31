#include "RotateOp.h"

#include "TrajectoryOpFormat.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace trajectory_algo
{
namespace
{
constexpr double kRotateNoOpAngleEps = 1e-9;

bool isRotateNoOp(const RobotInstruction::RotateParams& p)
{
	return std::abs(p.angleDeg) <= kRotateNoOpAngleEps
		&& std::abs(p.endAngleDeg) <= kRotateNoOpAngleEps;
}

bool isRotateInterpolated(const RobotInstruction::RotateParams& p)
{
	return std::abs(p.angleDeg - p.endAngleDeg) > kRotateNoOpAngleEps;
}
} // namespace

RobotInstruction::TrajectoryOpKind RotateOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Rotate;
}

const char* RotateOp::displayName(const bool chinese) const
{
	return chinese ? "旋转" : "Rotate";
}

TrajectoryOpCapability RotateOp::capabilities() const
{
	return TrajectoryOpCapability::PreviewPoseTransform | TrajectoryOpCapability::ApplyPoseTransform;
}

RobotInstruction::TrajectoryOpDescriptor RotateOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Rotate;
	op.scope = defaultScope;
	op.rotate.frame = RobotInstruction::TransformReferenceFrame::World;
	op.rotate.axisZ = 1.0;
	op.rotate.endAngleDeg = op.rotate.angleDeg;
	return op;
}

std::vector<TrajectoryOpParamField> RotateOp::paramFields() const
{
	return {
		enumParamField(
			"rotate.frame",
			"Frame",
			"坐标系",
			{ "0", "1" },
			{ "世界系", "物体系" },
			{ "World", "Body" },
			0,
			0,
			"transform"),
		doubleParamField("rotate.axisX", "Axis X", "轴 X", "", -1.0, 1.0, 0.001, 0.0, 1),
		doubleParamField("rotate.axisY", "Axis Y", "轴 Y", "", -1.0, 1.0, 0.001, 0.0, 2),
		doubleParamField("rotate.axisZ", "Axis Z", "轴 Z", "", -1.0, 1.0, 0.001, 1.0, 3),
		doubleParamField("rotate.angleDeg", "Angle(Start)", "角度(起点)", "°", -360.0, 360.0, 0.01, 0.0, 4),
		doubleParamField("rotate.endAngleDeg", "Angle(End)", "角度(终点)", "°", -360.0, 360.0, 0.01, 0.0, 5),
	};
}

bool RotateOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	if (isRotateNoOp(op.rotate))
	{
		return true;
	}
	const double len = std::sqrt(
		op.rotate.axisX * op.rotate.axisX + op.rotate.axisY * op.rotate.axisY
		+ op.rotate.axisZ * op.rotate.axisZ);
	if (len < 1e-9)
	{
		if (errMsg)
		{
			*errMsg = "rotation axis is zero";
		}
		return false;
	}
	return true;
}

std::string RotateOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	char buffer[256];
	if (isRotateInterpolated(op.rotate))
	{
		std::snprintf(
			buffer,
			sizeof(buffer),
			chinese ? "%s | %s | 起点%.2f° -> 终点%.2f° @(%.2f,%.2f,%.2f)"
					: "%s | %s | Start%.2f° -> End%.2f° @(%.2f,%.2f,%.2f)",
			displayName(chinese),
			frameLabel(op.rotate.frame, chinese).c_str(),
			op.rotate.angleDeg,
			op.rotate.endAngleDeg,
			op.rotate.axisX,
			op.rotate.axisY,
			op.rotate.axisZ);
	}
	else
	{
		std::snprintf(
			buffer,
			sizeof(buffer),
			chinese ? "%s | %s | %.2f° @(%.2f,%.2f,%.2f)"
					: "%s | %s | %.2f° @(%.2f,%.2f,%.2f)",
			displayName(chinese),
			frameLabel(op.rotate.frame, chinese).c_str(),
			op.rotate.angleDeg,
			op.rotate.axisX,
			op.rotate.axisY,
			op.rotate.axisZ);
	}
	return buffer;
}

bool RotateOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	if (isRotateNoOp(op.rotate))
	{
		return false;
	}
	out.kind = PreviewTransformStep::Kind::RotateOnly;
	out.targetIds.clear();
	for (const std::string& id : targetIds)
	{
		out.targetIds.insert(id);
	}
	out.rotate = op.rotate;
	return !out.targetIds.empty();
}

std::vector<TrajectoryApplyAction> RotateOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	if (isRotateNoOp(op.rotate))
	{
		return {};
	}
	TrajectoryApplyAction action{};
	action.kind = TrajectoryApplyActionKind::TransformSegment;
	action.transformOps = { op };
	return { action };
}

} // namespace trajectory_algo
