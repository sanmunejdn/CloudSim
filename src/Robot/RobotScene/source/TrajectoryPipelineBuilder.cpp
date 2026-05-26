#include "TrajectoryPipelineBuilder.h"

#include "InstructionProgramDocument.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"

#include <RigidTransform.h>

#include <cmath>

namespace RobotInstruction
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

engine::RigidTransform deltaFromDescriptor(const TrajectoryOpDescriptor& op)
{
	if (op.kind == TrajectoryOpKind::Translate)
	{
		return engine::RigidTransform::fromTranslationQuat(
			Eigen::Vector3d(op.translate.dxMm, op.translate.dyMm, op.translate.dzMm),
			Eigen::Quaterniond::Identity());
	}
	if (op.kind == TrajectoryOpKind::Rotate)
	{
		Eigen::Vector3d axis(op.rotate.axisX, op.rotate.axisY, op.rotate.axisZ);
		if (axis.norm() < 1e-9)
		{
			axis = Eigen::Vector3d::UnitZ();
		}
		axis.normalize();
		const double rad = op.rotate.angleDeg * kPi / 180.0;
		return engine::RigidTransform::fromTranslationQuat(
			Eigen::Vector3d::Zero(),
			Eigen::Quaterniond(Eigen::AngleAxisd(rad, axis)));
	}
	return engine::RigidTransform::identity();
}

engine::RigidTransform combinedTransformForOps(const std::vector<TrajectoryOpDescriptor>& ops)
{
	engine::RigidTransform combined = engine::RigidTransform::identity();
	for (const TrajectoryOpDescriptor& op : ops)
	{
		if (op.kind == TrajectoryOpKind::Translate || op.kind == TrajectoryOpKind::Rotate)
		{
			combined = deltaFromDescriptor(op).composeColumn(combined);
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
	engine::RigidTransform delta = engine::RigidTransform::identity();
	if (m_translate.dxMm != 0.0 || m_translate.dyMm != 0.0 || m_translate.dzMm != 0.0)
	{
		TrajectoryOpDescriptor tOp{};
		tOp.kind = TrajectoryOpKind::Translate;
		tOp.translate = m_translate;
		delta = deltaFromDescriptor(tOp).composeColumn(delta);
	}
	if (m_rotate.angleDeg != 0.0)
	{
		TrajectoryOpDescriptor rOp{};
		rOp.kind = TrajectoryOpKind::Rotate;
		rOp.rotate = m_rotate;
		delta = deltaFromDescriptor(rOp).composeColumn(delta);
	}
	const engine::RigidTransform updated = delta.composeColumn(target);
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
	std::unique_ptr<IMotionPoseQuery> chain = std::make_unique<DefaultMotionPoseQuery>();
	if (!program)
	{
		return chain;
	}
	RobotProgramCatalog catalog;
	for (const TrajectoryOpDescriptor& op : ops)
	{
		if (op.kind != TrajectoryOpKind::Translate && op.kind != TrajectoryOpKind::Rotate)
		{
			continue;
		}
		std::unordered_set<std::string> ids;
		for (const std::string& id : catalog.resolveOpScopeInstructionIds(op.scope, *program))
		{
			ids.insert(id);
		}
		if (ids.empty())
		{
			continue;
		}
		TranslateParams translate{};
		RotateParams rotate{};
		if (op.kind == TrajectoryOpKind::Translate)
		{
			translate = op.translate;
		}
		else if (op.kind == TrajectoryOpKind::Rotate)
		{
			rotate = op.rotate;
		}
		chain = std::make_unique<TransformMotionPoseQuery>(
			std::move(chain),
			std::move(ids),
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
	(void)doc;
	std::vector<ProgramEditStack::CommandPtr> out;
	if (!m_program)
	{
		if (errMsg)
		{
			*errMsg = "no program context";
		}
		return out;
	}
	RobotProgramCatalog catalog;
	std::vector<TrajectoryOpDescriptor> transformOps;
	for (const TrajectoryOpDescriptor& op : m_ops)
	{
		if (op.kind == TrajectoryOpKind::Translate || op.kind == TrajectoryOpKind::Rotate)
		{
			transformOps.push_back(op);
			continue;
		}
		const std::vector<std::string> ids = catalog.resolveOpScopeInstructionIds(op.scope, *m_program);
		if (ids.empty())
		{
			continue;
		}
		if (op.kind == TrajectoryOpKind::Delete)
		{
			for (const std::string& id : ids)
			{
				out.push_back(std::make_shared<RemoveInstructionCommand>(id));
			}
		}
		else if (op.kind == TrajectoryOpKind::Duplicate)
		{
			for (const std::string& id : ids)
			{
				out.push_back(std::make_shared<DuplicateInstructionCommand>(id, 0));
			}
		}
	}
	for (const TrajectoryOpDescriptor& op : transformOps)
	{
		const std::vector<std::string> ids = catalog.resolveOpScopeInstructionIds(op.scope, *m_program);
		if (!ids.empty())
		{
			out.push_back(std::make_shared<TransformMotionSegmentCommand>(
				ids,
				std::vector<TrajectoryOpDescriptor>{ op }));
		}
	}
	if (out.empty() && errMsg)
	{
		*errMsg = "no applicable operations";
	}
	return out;
}

} // namespace RobotInstruction
