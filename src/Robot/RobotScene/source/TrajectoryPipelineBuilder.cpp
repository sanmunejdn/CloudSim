#include "TrajectoryPipelineBuilder.h"

#include "InstructionProgramDocument.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"
#include "RobotProgramCatalog.h"
#include "TrajectoryApplyActionConverter.h"

#include <ITrajectoryOp.h>
#include <TrajectoryApplyAction.h>
#include <TrajectoryOpRegistry.h>
#include <TrajectoryTransformMath.h>

#include <RigidTransform.h>

namespace RobotInstruction
{
namespace
{

engine::RigidTransform deltaFromDescriptor(const TrajectoryOpDescriptor& op)
{
	if (op.kind == TrajectoryOpKind::Translate)
	{
		return trajectory_algo::rigidDeltaFromTranslate(op.translate);
	}
	if (op.kind == TrajectoryOpKind::Rotate)
	{
		return trajectory_algo::rigidDeltaFromRotate(op.rotate);
	}
	return engine::RigidTransform::identity();
}

RobotInstruction::TransformReferenceFrame frameForDescriptor(const TrajectoryOpDescriptor& op)
{
	if (op.kind == TrajectoryOpKind::Translate)
	{
		return op.translate.frame;
	}
	if (op.kind == TrajectoryOpKind::Rotate)
	{
		return op.rotate.frame;
	}
	return TransformReferenceFrame::World;
}

engine::RigidTransform combinedTransformForOps(const std::vector<TrajectoryOpDescriptor>& ops)
{
	engine::RigidTransform combined = engine::RigidTransform::identity();
	for (const TrajectoryOpDescriptor& op : ops)
	{
		if (op.kind != TrajectoryOpKind::Translate && op.kind != TrajectoryOpKind::Rotate)
		{
			continue;
		}
		const engine::RigidTransform delta = deltaFromDescriptor(op);
		const TransformReferenceFrame frame = frameForDescriptor(op);
		if (frame == TransformReferenceFrame::Body)
		{
			combined = combined.composeColumn(delta);
		}
		else
		{
			combined = delta.composeColumn(combined);
		}
	}
	return combined;
}

} // namespace

bool DefaultMotionPoseQuery::queryMotionPose(const Base& ins, Vec3& poseMm, Vec3& eulerDeg) const
{
	if (!ins.hasPoseProperty())
	{
		return false;
	}
	engine::RigidTransform target = engine::RigidTransform::identity();
	if (!readTargetTransformFromInstruction(ins, target))
	{
		poseMm = ins.pose();
		eulerDeg = ins.eulerDeg();
		return true;
	}
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	target.translationMm(px, py, pz);
	target.eulerDegForDisplay(ex, ey, ez);
	poseMm = Vec3{ px, py, pz };
	eulerDeg = Vec3{ ex, ey, ez };
	return true;
}

TransformMotionPoseQuery::TransformMotionPoseQuery(
	std::unique_ptr<IMotionPoseQuery> inner,
	std::unordered_set<std::string> targetIds,
	TranslateParams translate,
	RotateParams rotate)
	: m_inner(std::move(inner))
	, m_targetIds(std::move(targetIds))
	, m_translate(translate)
	, m_rotate(rotate)
{
}

bool TransformMotionPoseQuery::queryMotionPose(const Base& ins, Vec3& poseMm, Vec3& eulerDeg) const
{
	if (!m_inner || !m_inner->queryMotionPose(ins, poseMm, eulerDeg))
	{
		return false;
	}
	if (!isMotionWaypointType(ins.type()) || m_targetIds.count(ins.id()) == 0)
	{
		return true;
	}
	engine::RigidTransform target = engine::RigidTransform::identity();
	if (!readTargetTransformFromInstruction(ins, target))
	{
		target = engine::RigidTransform::fromTranslationEulerDeg(
			poseMm.x,
			poseMm.y,
			poseMm.z,
			eulerDeg.x,
			eulerDeg.y,
			eulerDeg.z);
	}
	engine::RigidTransform updated = target;
	if (m_translate.dxMm != 0.0 || m_translate.dyMm != 0.0 || m_translate.dzMm != 0.0)
	{
		TrajectoryOpDescriptor tOp{};
		tOp.kind = TrajectoryOpKind::Translate;
		tOp.translate = m_translate;
		const engine::RigidTransform tDelta = deltaFromDescriptor(tOp);
		updated = trajectory_algo::applyTransformDelta(updated, tDelta, m_translate.frame);
	}
	if (m_rotate.angleDeg != 0.0)
	{
		TrajectoryOpDescriptor rOp{};
		rOp.kind = TrajectoryOpKind::Rotate;
		rOp.rotate = m_rotate;
		const engine::RigidTransform rDelta = deltaFromDescriptor(rOp);
		updated = trajectory_algo::applyTransformDelta(updated, rDelta, m_rotate.frame);
	}
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	updated.translationMm(px, py, pz);
	updated.eulerDegForDisplay(ex, ey, ez);
	poseMm = Vec3{ px, py, pz };
	eulerDeg = Vec3{ ex, ey, ez };
	return true;
}

std::unique_ptr<IMotionPoseQuery> buildPreviewPoseQueryChain(
	const std::vector<std::shared_ptr<Base>>& rootSteps,
	const RobotProgram* program,
	const std::vector<TrajectoryOpDescriptor>& ops)
{
	(void)rootSteps;
	trajectory_algo::ensureTrajectoryOpBuiltinsRegistered();
	std::unique_ptr<IMotionPoseQuery> chain = std::make_unique<DefaultMotionPoseQuery>();
	if (!program)
	{
		return chain;
	}
	RobotProgramCatalog catalog;
	for (const TrajectoryOpDescriptor& op : ops)
	{
		const trajectory_algo::ITrajectoryOp* algo =
			trajectory_algo::TrajectoryOpRegistry::instance().get(op.kind);
		if (!algo || !trajectory_algo::hasCapability(
				algo->capabilities(),
				trajectory_algo::TrajectoryOpCapability::PreviewPoseTransform))
		{
			continue;
		}
		std::vector<std::string> scopeIds = catalog.resolveOpScopeInstructionIds(op.scope, *program);
		if (op.scope.kind == OpScope::Kind::Group)
		{
			scopeIds = catalog.expandToMotionWaypointIds(*program, scopeIds);
		}
		trajectory_algo::PreviewTransformStep step{};
		if (!algo->contributePreviewTransform(op, scopeIds, step) || step.targetIds.empty())
		{
			continue;
		}
		TranslateParams translate{};
		RotateParams rotate{};
		if (step.kind == trajectory_algo::PreviewTransformStep::Kind::TranslateOnly)
		{
			translate = step.translate;
		}
		else if (step.kind == trajectory_algo::PreviewTransformStep::Kind::RotateOnly)
		{
			rotate = step.rotate;
		}
		chain = std::make_unique<TransformMotionPoseQuery>(
			std::move(chain),
			std::move(step.targetIds),
			translate,
			rotate);
	}
	return chain;
}

void TrajectoryPipelineBuilder::setProgramContext(const RobotProgram* program)
{
	m_program = program;
}

void TrajectoryPipelineBuilder::setOps(std::vector<TrajectoryOpDescriptor> ops)
{
	m_ops = std::move(ops);
}

std::unique_ptr<IMotionPoseQuery> TrajectoryPipelineBuilder::buildPreviewPoseQuery(
	const std::vector<std::shared_ptr<Base>>& rootSteps) const
{
	return buildPreviewPoseQueryChain(rootSteps, m_program, m_ops);
}

std::vector<ProgramEditStack::CommandPtr> TrajectoryPipelineBuilder::buildApplyCommands(
	InstructionProgramDocument& doc,
	std::string* errMsg) const
{
	trajectory_algo::ensureTrajectoryOpBuiltinsRegistered();
	std::vector<ProgramEditStack::CommandPtr> out;
	if (!m_program)
	{
		if (errMsg)
		{
			*errMsg = "no program context";
		}
		return out;
	}
	trajectory_algo::TrajectoryOpContext ctx{};
	ctx.program = m_program;
	std::vector<trajectory_algo::TrajectoryApplyAction> actions;
	for (const TrajectoryOpDescriptor& op : m_ops)
	{
		const trajectory_algo::ITrajectoryOp* algo =
			trajectory_algo::TrajectoryOpRegistry::instance().get(op.kind);
		if (!algo)
		{
			continue;
		}
		std::vector<trajectory_algo::TrajectoryApplyAction> built = algo->buildApplyActions(ctx, op);
		actions.insert(actions.end(), built.begin(), built.end());
	}
	return convertApplyActionsToCommands(actions, *m_program, doc, errMsg);
}

} // namespace RobotInstruction
