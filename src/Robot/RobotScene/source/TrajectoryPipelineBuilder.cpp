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

#include <cmath>
#include <unordered_map>

namespace RobotInstruction
{
namespace
{
constexpr double kNoOpEps = 1e-9;
constexpr double kAxisMergeEps = 1e-6;

bool scopeEquals(const OpScope& a, const OpScope& b)
{
	if (a.kind != b.kind || a.groupId != b.groupId || a.pointFrom != b.pointFrom || a.pointTo != b.pointTo)
	{
		return false;
	}
	return a.instructionIds == b.instructionIds;
}

bool rotateAxisAlmostEquals(const RotateParams& a, const RotateParams& b)
{
	return std::abs(a.axisX - b.axisX) <= kAxisMergeEps
		&& std::abs(a.axisY - b.axisY) <= kAxisMergeEps
		&& std::abs(a.axisZ - b.axisZ) <= kAxisMergeEps;
}

bool isTranslateNoOp(const TranslateParams& p)
{
	return std::abs(p.dxMm) <= kNoOpEps
		&& std::abs(p.dyMm) <= kNoOpEps
		&& std::abs(p.dzMm) <= kNoOpEps
		&& std::abs(p.endDxMm) <= kNoOpEps
		&& std::abs(p.endDyMm) <= kNoOpEps
		&& std::abs(p.endDzMm) <= kNoOpEps;
}

bool isTranslateInterpolated(const TranslateParams& p)
{
	return std::abs(p.dxMm - p.endDxMm) > kNoOpEps
		|| std::abs(p.dyMm - p.endDyMm) > kNoOpEps
		|| std::abs(p.dzMm - p.endDzMm) > kNoOpEps;
}

bool isRotateNoOp(const RotateParams& p)
{
	return std::abs(p.angleDeg) <= kNoOpEps
		&& std::abs(p.endAngleDeg) <= kNoOpEps;
}

bool isRotateInterpolated(const RotateParams& p)
{
	return std::abs(p.angleDeg - p.endAngleDeg) > kNoOpEps;
}

double lerp(const double a, const double b, const double t)
{
	return a + (b - a) * t;
}

TranslateParams interpolatedTranslate(const TranslateParams& p, const double t)
{
	TranslateParams out = p;
	out.dxMm = lerp(p.dxMm, p.endDxMm, t);
	out.dyMm = lerp(p.dyMm, p.endDyMm, t);
	out.dzMm = lerp(p.dzMm, p.endDzMm, t);
	out.endDxMm = out.dxMm;
	out.endDyMm = out.dyMm;
	out.endDzMm = out.dzMm;
	return out;
}

RotateParams interpolatedRotate(const RotateParams& p, const double t)
{
	RotateParams out = p;
	out.angleDeg = lerp(p.angleDeg, p.endAngleDeg, t);
	out.endAngleDeg = out.angleDeg;
	return out;
}

std::vector<std::string> orderedTargetIds(
	const std::vector<std::string>& scopeIds,
	const std::unordered_set<std::string>& targetIds)
{
	std::vector<std::string> out;
	out.reserve(targetIds.size());
	std::unordered_set<std::string> seen;
	for (const std::string& id : scopeIds)
	{
		if (targetIds.count(id) == 0 || !seen.insert(id).second)
		{
			continue;
		}
		out.push_back(id);
	}
	for (const std::string& id : targetIds)
	{
		if (seen.insert(id).second)
		{
			out.push_back(id);
		}
	}
	return out;
}

bool isDescriptorNoOp(const TrajectoryOpDescriptor& op)
{
	if (op.kind == TrajectoryOpKind::Translate)
	{
		return isTranslateNoOp(op.translate);
	}
	if (op.kind == TrajectoryOpKind::Rotate)
	{
		return isRotateNoOp(op.rotate);
	}
	return false;
}

bool canMergeTranslate(const TrajectoryOpDescriptor& prev, const TrajectoryOpDescriptor& cur)
{
	return prev.kind == TrajectoryOpKind::Translate && cur.kind == TrajectoryOpKind::Translate
		&& scopeEquals(prev.scope, cur.scope) && prev.translate.frame == cur.translate.frame
		&& !isTranslateInterpolated(prev.translate) && !isTranslateInterpolated(cur.translate);
}

bool canMergeRotate(const TrajectoryOpDescriptor& prev, const TrajectoryOpDescriptor& cur)
{
	return prev.kind == TrajectoryOpKind::Rotate && cur.kind == TrajectoryOpKind::Rotate
		&& scopeEquals(prev.scope, cur.scope) && prev.rotate.frame == cur.rotate.frame
		&& rotateAxisAlmostEquals(prev.rotate, cur.rotate)
		&& !isRotateInterpolated(prev.rotate) && !isRotateInterpolated(cur.rotate);
}

std::vector<TrajectoryOpDescriptor> normalizeOps(const std::vector<TrajectoryOpDescriptor>& ops)
{
	std::vector<TrajectoryOpDescriptor> out;
	out.reserve(ops.size());
	for (const TrajectoryOpDescriptor& op : ops)
	{
		if (isDescriptorNoOp(op))
		{
			continue;
		}
		if (out.empty())
		{
			out.push_back(op);
			continue;
		}
		TrajectoryOpDescriptor& prev = out.back();
		if (canMergeTranslate(prev, op))
		{
			prev.translate.dxMm += op.translate.dxMm;
			prev.translate.dyMm += op.translate.dyMm;
			prev.translate.dzMm += op.translate.dzMm;
			prev.translate.endDxMm += op.translate.endDxMm;
			prev.translate.endDyMm += op.translate.endDyMm;
			prev.translate.endDzMm += op.translate.endDzMm;
			if (isTranslateNoOp(prev.translate))
			{
				out.pop_back();
			}
			continue;
		}
		if (canMergeRotate(prev, op))
		{
			prev.rotate.angleDeg += op.rotate.angleDeg;
			prev.rotate.endAngleDeg += op.rotate.endAngleDeg;
			if (isRotateNoOp(prev.rotate))
			{
				out.pop_back();
			}
			continue;
		}
		out.push_back(op);
	}
	return out;
}

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

class AxisReversePoseQuery final : public IMotionPoseQuery
{
public:
	AxisReversePoseQuery(
		std::unique_ptr<IMotionPoseQuery> inner,
		std::unordered_set<std::string> targetIds,
		const int mirrorAxis)
		: m_inner(std::move(inner))
		, m_targetIds(std::move(targetIds))
		, m_mirrorAxis(mirrorAxis)
	{
	}

	bool queryMotionPose(const Base& ins, Vec3& poseMm, Vec3& eulerDeg) const override
	{
		if (!m_inner || !m_inner->queryMotionPose(ins, poseMm, eulerDeg))
		{
			return false;
		}
		if (!isMotionWaypointType(ins.type()) || m_targetIds.count(ins.id()) == 0)
		{
			return true;
		}
		engine::RigidTransform target = engine::RigidTransform::fromTranslationEulerDeg(
			poseMm.x, poseMm.y, poseMm.z, eulerDeg.x, eulerDeg.y, eulerDeg.z);
		Eigen::Matrix3d rot = target.rotation().toRotationMatrix();
		Eigen::Vector3d axes[3] = { rot.col(0), rot.col(1), rot.col(2) };
		const int reversedAxis = std::max(0, std::min(2, m_mirrorAxis));
		const int keptAxis = (reversedAxis + 1) % 3;
		const int rebuiltAxis = 3 - reversedAxis - keptAxis;
		axes[reversedAxis] = -axes[reversedAxis];
		axes[rebuiltAxis] = axes[reversedAxis].cross(axes[keptAxis]);
		if (axes[rebuiltAxis].norm() < 1e-9)
		{
			return true;
		}
		axes[rebuiltAxis].normalize();
		axes[keptAxis] = axes[rebuiltAxis].cross(axes[reversedAxis]);
		if (axes[keptAxis].norm() < 1e-9)
		{
			return true;
		}
		axes[reversedAxis].normalize();
		axes[keptAxis].normalize();
		Eigen::Matrix3d updatedRot;
		updatedRot.col(0) = axes[0];
		updatedRot.col(1) = axes[1];
		updatedRot.col(2) = axes[2];
		engine::RigidTransform updated = engine::RigidTransform::fromTranslationQuat(
			target.translationMm(),
			Eigen::Quaterniond(updatedRot));
		double ex = 0.0;
		double ey = 0.0;
		double ez = 0.0;
		updated.eulerDegForDisplay(ex, ey, ez);
		eulerDeg = Vec3{ ex, ey, ez };
		return true;
	}

private:
	std::unique_ptr<IMotionPoseQuery> m_inner;
	std::unordered_set<std::string> m_targetIds;
	int m_mirrorAxis = 0;
};

class FixedOrientationPoseQuery final : public IMotionPoseQuery
{
public:
	FixedOrientationPoseQuery(
		std::unique_ptr<IMotionPoseQuery> inner,
		std::unordered_set<std::string> targetIds,
		std::string referenceId,
		const std::vector<std::shared_ptr<Base>>& rootSteps)
		: m_inner(std::move(inner))
		, m_targetIds(std::move(targetIds))
		, m_referenceId(std::move(referenceId))
	{
		std::vector<std::shared_ptr<Base>> rootCopy = rootSteps;
		std::vector<std::shared_ptr<Base>> flat;
		flattenInstructionsRecursive(rootCopy, flat);
		for (const auto& ins : flat)
		{
			if (ins)
			{
				m_idMap.emplace(ins->id(), ins.get());
			}
		}
	}

	bool queryMotionPose(const Base& ins, Vec3& poseMm, Vec3& eulerDeg) const override
	{
		if (!m_inner || !m_inner->queryMotionPose(ins, poseMm, eulerDeg))
		{
			return false;
		}
		if (!isMotionWaypointType(ins.type()) || m_targetIds.count(ins.id()) == 0)
		{
			return true;
		}
		if (!m_refResolved)
		{
			m_refResolved = true;
			const auto it = m_idMap.find(m_referenceId);
			if (it != m_idMap.end() && it->second)
			{
				Vec3 refPose{};
				Vec3 refEuler{};
				if (m_inner->queryMotionPose(*it->second, refPose, refEuler))
				{
					m_refEuler = refEuler;
					m_hasRefEuler = true;
				}
			}
		}
		if (m_hasRefEuler)
		{
			eulerDeg = m_refEuler;
		}
		return true;
	}

private:
	std::unique_ptr<IMotionPoseQuery> m_inner;
	std::unordered_set<std::string> m_targetIds;
	std::string m_referenceId;
	std::unordered_map<std::string, const Base*> m_idMap;
	mutable bool m_refResolved = false;
	mutable bool m_hasRefEuler = false;
	mutable Vec3 m_refEuler{};
};

std::unique_ptr<IMotionPoseQuery> buildPreviewPoseQueryChain(
	const std::vector<std::shared_ptr<Base>>& rootSteps,
	const RobotProgram* program,
	const std::vector<TrajectoryOpDescriptor>& ops)
{
	trajectory_algo::ensureTrajectoryOpBuiltinsRegistered();
	std::unique_ptr<IMotionPoseQuery> chain = std::make_unique<DefaultMotionPoseQuery>();
	if (!program)
	{
		return chain;
	}
	const std::vector<TrajectoryOpDescriptor> normalizedOps = normalizeOps(ops);
	RobotProgramCatalog catalog;
	for (const TrajectoryOpDescriptor& op : normalizedOps)
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
		const std::vector<std::string> orderedIds = orderedTargetIds(scopeIds, step.targetIds);
		if (orderedIds.empty())
		{
			continue;
		}
		if (step.kind == trajectory_algo::PreviewTransformStep::Kind::AxisReverse)
		{
			chain = std::make_unique<AxisReversePoseQuery>(
				std::move(chain),
				std::move(step.targetIds),
				step.mirrorAxis);
			continue;
		}
		if (step.kind == trajectory_algo::PreviewTransformStep::Kind::FixedOrientationToFirst)
		{
			std::string referenceId = step.referenceId.empty() ? orderedIds.front() : step.referenceId;
			chain = std::make_unique<FixedOrientationPoseQuery>(
				std::move(chain),
				std::move(step.targetIds),
				std::move(referenceId),
				rootSteps);
			continue;
		}
		const size_t n = orderedIds.size();
		const bool interpolateTranslate = step.kind == trajectory_algo::PreviewTransformStep::Kind::TranslateOnly
			&& isTranslateInterpolated(step.translate) && n > 1;
		const bool interpolateRotate = step.kind == trajectory_algo::PreviewTransformStep::Kind::RotateOnly
			&& isRotateInterpolated(step.rotate) && n > 1;
		if (!interpolateTranslate && !interpolateRotate)
		{
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
			continue;
		}
		for (size_t i = 0; i < orderedIds.size(); ++i)
		{
			const double t = orderedIds.size() <= 1 ? 0.0 : static_cast<double>(i) / static_cast<double>(orderedIds.size() - 1);
			std::unordered_set<std::string> singleId = { orderedIds[i] };
			TranslateParams translate{};
			RotateParams rotate{};
			if (interpolateTranslate)
			{
				translate = interpolatedTranslate(step.translate, t);
			}
			if (interpolateRotate)
			{
				rotate = interpolatedRotate(step.rotate, t);
			}
			chain = std::make_unique<TransformMotionPoseQuery>(
				std::move(chain),
				std::move(singleId),
				translate,
				rotate);
		}
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
	const std::vector<TrajectoryOpDescriptor> normalizedOps = normalizeOps(m_ops);
	for (const TrajectoryOpDescriptor& op : normalizedOps)
	{
		const trajectory_algo::ITrajectoryOp* algo =
			trajectory_algo::TrajectoryOpRegistry::instance().get(op.kind);
		if (!algo)
		{
			continue;
		}
		std::string validateErr;
		if (!algo->validate(op, &validateErr))
		{
			if (errMsg)
			{
				*errMsg = validateErr.empty() ? "invalid trajectory operation" : validateErr;
			}
			return {};
		}
		std::vector<trajectory_algo::TrajectoryApplyAction> built = algo->buildApplyActions(ctx, op);
		actions.insert(actions.end(), built.begin(), built.end());
	}
	return convertApplyActionsToCommands(actions, *m_program, doc, errMsg);
}

} // namespace RobotInstruction
