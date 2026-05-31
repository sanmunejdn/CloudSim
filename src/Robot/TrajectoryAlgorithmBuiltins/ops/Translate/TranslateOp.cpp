#include "TranslateOp.h"

#include "TrajectoryOpFormat.h"

#include <cmath>
#include <cstdio>
#include <string>

namespace trajectory_algo
{
namespace
{
constexpr double kTranslateNoOpEps = 1e-9;

bool isTranslateNoOp(const RobotInstruction::TranslateParams& p)
{
	return std::abs(p.dxMm) <= kTranslateNoOpEps
		&& std::abs(p.dyMm) <= kTranslateNoOpEps
		&& std::abs(p.dzMm) <= kTranslateNoOpEps
		&& std::abs(p.endDxMm) <= kTranslateNoOpEps
		&& std::abs(p.endDyMm) <= kTranslateNoOpEps
		&& std::abs(p.endDzMm) <= kTranslateNoOpEps;
}

bool isTranslateInterpolated(const RobotInstruction::TranslateParams& p)
{
	return std::abs(p.dxMm - p.endDxMm) > kTranslateNoOpEps
		|| std::abs(p.dyMm - p.endDyMm) > kTranslateNoOpEps
		|| std::abs(p.dzMm - p.endDzMm) > kTranslateNoOpEps;
}
} // namespace

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
	op.translate.endDxMm = op.translate.dxMm;
	op.translate.endDyMm = op.translate.dyMm;
	op.translate.endDzMm = op.translate.dzMm;
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
		doubleParamField("translate.dxMm", "ΔX(Start)", "ΔX(起点)", "mm", -1e5, 1e5, 0.01, 0.0, 1),
		doubleParamField("translate.dyMm", "ΔY(Start)", "ΔY(起点)", "mm", -1e5, 1e5, 0.01, 0.0, 2),
		doubleParamField("translate.dzMm", "ΔZ(Start)", "ΔZ(起点)", "mm", -1e5, 1e5, 0.01, 0.0, 3),
		doubleParamField("translate.endDxMm", "ΔX(End)", "ΔX(终点)", "mm", -1e5, 1e5, 0.01, 0.0, 4),
		doubleParamField("translate.endDyMm", "ΔY(End)", "ΔY(终点)", "mm", -1e5, 1e5, 0.01, 0.0, 5),
		doubleParamField("translate.endDzMm", "ΔZ(End)", "ΔZ(终点)", "mm", -1e5, 1e5, 0.01, 0.0, 6),
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
	if (isTranslateInterpolated(op.translate))
	{
		std::snprintf(
			buffer,
			sizeof(buffer),
			chinese ? "%s | %s | 起点Δ(%.2f,%.2f,%.2f) -> 终点Δ(%.2f,%.2f,%.2f) mm"
					: "%s | %s | StartΔ(%.2f,%.2f,%.2f) -> EndΔ(%.2f,%.2f,%.2f) mm",
			displayName(chinese),
			frameStr.c_str(),
			op.translate.dxMm,
			op.translate.dyMm,
			op.translate.dzMm,
			op.translate.endDxMm,
			op.translate.endDyMm,
			op.translate.endDzMm);
	}
	else
	{
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
	}
	return buffer;
}

bool TranslateOp::contributePreviewTransform(
	const RobotInstruction::TrajectoryOpDescriptor& op,
	const std::vector<std::string>& targetIds,
	PreviewTransformStep& out) const
{
	if (isTranslateNoOp(op.translate))
	{
		return false;
	}
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
	if (isTranslateNoOp(op.translate))
	{
		return {};
	}
	TrajectoryApplyAction action{};
	action.kind = TrajectoryApplyActionKind::TransformSegment;
	action.transformOps = { op };
	return { action };
}

} // namespace trajectory_algo
