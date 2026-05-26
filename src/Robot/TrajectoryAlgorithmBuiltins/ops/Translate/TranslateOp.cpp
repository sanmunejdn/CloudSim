#include "TranslateOp.h"

#include "TrajectoryOpFormat.h"

#include <cstdio>
#include <string>

namespace trajectory_algo
{

RobotInstruction::TrajectoryOpKind TranslateOp::kind() const
{
	return RobotInstruction::TrajectoryOpKind::Translate;
}

const char* TranslateOp::displayName(const bool chinese) const
{
	return chinese ? "平移" : "Translate";
}

TrajectoryOpCapability TranslateOp::capabilities() const
{
	return TrajectoryOpCapability::PreviewPoseTransform | TrajectoryOpCapability::ApplyPoseTransform;
}

RobotInstruction::TrajectoryOpDescriptor TranslateOp::makeDefaultDescriptor(
	const RobotInstruction::OpScope& defaultScope) const
{
	RobotInstruction::TrajectoryOpDescriptor op{};
	op.kind = RobotInstruction::TrajectoryOpKind::Translate;
	op.scope = defaultScope;
	op.translate.frame = RobotInstruction::TransformReferenceFrame::World;
	return op;
}

std::vector<TrajectoryOpParamField> TranslateOp::paramFields() const
{
	return {
		enumParamField(
			"translate.frame",
			"Frame",
			"坐标系",
			{ "0", "1" },
			{ "世界系", "物体系" },
			{ "World", "Body" },
			0,
			0,
			"transform"),
		doubleParamField("translate.dxMm", "ΔX", "ΔX", "mm", -1e5, 1e5, 0.01, 0.0, 1),
		doubleParamField("translate.dyMm", "ΔY", "ΔY", "mm", -1e5, 1e5, 0.01, 0.0, 2),
		doubleParamField("translate.dzMm", "ΔZ", "ΔZ", "mm", -1e5, 1e5, 0.01, 0.0, 3),
	};
}

bool TranslateOp::validate(const RobotInstruction::TrajectoryOpDescriptor& op, std::string* errMsg) const
{
	(void)op;
	(void)errMsg;
	return true;
}

std::string TranslateOp::formatSummary(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const bool chinese) const
{
	const std::string frameStr = frameLabel(op.translate.frame, chinese);
	char buffer[512];
	std::snprintf(
		buffer,
		sizeof(buffer),
		chinese ? "%s | %s | Δ(%.2f,%.2f,%.2f) mm"
				: "%s | %s | Δ(%.2f,%.2f,%.2f) mm",
		displayName(chinese),
		frameStr.c_str(),
		op.translate.dxMm,
		op.translate.dyMm,
		op.translate.dzMm);
	return buffer;
}

bool TranslateOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	out.kind = PreviewTransformStep::Kind::TranslateOnly;
	out.targetIds.clear();
	for (const std::string& id : targetIds)
	{
		out.targetIds.insert(id);
	}
	out.translate = op.translate;
	return !out.targetIds.empty();
}

std::vector<TrajectoryApplyAction> TranslateOp::buildApplyActions(
	const TrajectoryOpContext& ctx,
	const RobotInstruction::TrajectoryOpDescriptor& op) const
{
	(void)ctx;
	TrajectoryApplyAction action{};
	action.kind = TrajectoryApplyActionKind::TransformSegment;
	action.transformOps = { op };
	return { action };
}

} // namespace trajectory_algo
