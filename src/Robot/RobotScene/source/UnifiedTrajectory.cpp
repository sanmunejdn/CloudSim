#include "UnifiedTrajectory.h"

#include "RobotProgramCatalog.h"
#include "RobotInstructionFactory.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"

#include <TrajectoryTransformMath.h>
#include <TrajectoryUnifiedScope.h>

#include <RigidTransform.h>

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <unordered_set>

namespace RobotInstruction
{
namespace
{
double lerp(const double a, const double b, const double t)
{
	return a + (b - a) * t;
}

TrajectoryOpDescriptor interpolateDescriptor(const TrajectoryOpDescriptor& op, const double t)
{
	TrajectoryOpDescriptor out = op;
	if (op.kind == TrajectoryOpKind::Translate)
	{
		out.translate.dxMm = lerp(op.translate.dxMm, op.translate.endDxMm, t);
		out.translate.dyMm = lerp(op.translate.dyMm, op.translate.endDyMm, t);
		out.translate.dzMm = lerp(op.translate.dzMm, op.translate.endDzMm, t);
		out.translate.endDxMm = out.translate.dxMm;
		out.translate.endDyMm = out.translate.dyMm;
		out.translate.endDzMm = out.translate.dzMm;
	}
	else if (op.kind == TrajectoryOpKind::Rotate)
	{
		out.rotate.angleDeg = lerp(op.rotate.angleDeg, op.rotate.endAngleDeg, t);
		out.rotate.endAngleDeg = out.rotate.angleDeg;
	}
	return out;
}

engine::RigidTransform rigidFromPoint(const UnifiedTrajectoryPoint& p)
{
	return engine::RigidTransform::fromTranslationEulerDeg(
		p.poseMm.x,
		p.poseMm.y,
		p.poseMm.z,
		p.eulerDeg.x,
		p.eulerDeg.y,
		p.eulerDeg.z);
}

void pointFromRigid(const engine::RigidTransform& tf, UnifiedTrajectoryPoint& p)
{
	double px = 0.0;
	double py = 0.0;
	double pz = 0.0;
	double ex = 0.0;
	double ey = 0.0;
	double ez = 0.0;
	tf.translationMm(px, py, pz);
	tf.eulerDegForDisplay(ex, ey, ez);
	p.poseMm = Vec3{ px, py, pz };
	p.eulerDeg = Vec3{ ex, ey, ez };
}

Eigen::Vector3d tangentDirection(const UnifiedTrajectory& traj, const size_t index)
{
	if (traj.points.size() < 2)
	{
		return Eigen::Vector3d::UnitX();
	}
	const size_t prev = index == 0 ? 0 : index - 1;
	const size_t next = index + 1 >= traj.points.size() ? traj.points.size() - 1 : index + 1;
	Eigen::Vector3d dir(
		traj.points[next].poseMm.x - traj.points[prev].poseMm.x,
		traj.points[next].poseMm.y - traj.points[prev].poseMm.y,
		traj.points[next].poseMm.z - traj.points[prev].poseMm.z);
	if (dir.norm() < 1e-9)
	{
		return Eigen::Vector3d::UnitX();
	}
	return dir.normalized();
}

Eigen::Vector3d toolZDirection(const UnifiedTrajectoryPoint& point)
{
	const engine::RigidTransform tf = rigidFromPoint(point);
	Eigen::Matrix3d rot = tf.rotation().toRotationMatrix();
	Eigen::Vector3d z = rot.col(2);
	if (z.norm() < 1e-9)
	{
		return Eigen::Vector3d::UnitZ();
	}
	return z.normalized();
}

Eigen::Vector3d directionByMode(
	const UnifiedTrajectory& traj,
	const size_t index,
	const ApproachDirectionMode mode)
{
	switch (mode)
	{
	case ApproachDirectionMode::PathTangent:
		return tangentDirection(traj, index);
	case ApproachDirectionMode::ToolZ:
		return toolZDirection(traj.points[index]);
	case ApproachDirectionMode::SurfaceNormal:
	default:
		// 兜底沿工具 Z，避免法向不可得时整块失效
		return toolZDirection(traj.points[index]);
	}
}

bool matchesSegmentSelection(const SegmentSelectMode mode, const int from, const int to)
{
	if (mode == SegmentSelectMode::AllSegments)
	{
		return true;
	}
	return from <= 1 && to >= 1;
}

bool unifiedPointFromMotionInstruction(const Base& base, UnifiedTrajectoryPoint& out)
{
	if (!isMotionWaypointType(base.type()))
	{
		return false;
	}
	engine::RigidTransform target = engine::RigidTransform::identity();
	if (!readTargetTransformFromInstruction(base, target))
	{
		return false;
	}
	pointFromRigid(target, out);
	out.sourceInstructionId = base.id();
	out.blendRadiusMm = base.blendRadius();
	out.speedMmPerSec = base.speed();
	out.reachable = true;
	return true;
}

void insertApproachPoint(
	UnifiedTrajectory& traj,
	const ApproachParams& params,
	const OpScope& scope,
	const RobotProgram* program)
{
	if (traj.points.empty() || !matchesSegmentSelection(params.segmentSelectMode, params.segmentFrom, params.segmentTo))
	{
		return;
	}
	const std::vector<std::size_t> indices =
		trajectory_algo::resolveScopedPointIndices(traj, scope, program);
	if (indices.empty())
	{
		return;
	}
	const std::size_t anchor = indices.front();
	UnifiedTrajectoryPoint point = traj.points[anchor];
	const Eigen::Vector3d dir = directionByMode(traj, anchor, params.directionMode);
	point.poseMm.x -= dir.x() * params.distanceMm;
	point.poseMm.y -= dir.y() * params.distanceMm;
	point.poseMm.z -= dir.z() * params.distanceMm;
	point.sourceInstructionId.clear();
	if (params.overrideSpeedEnabled && params.speedMmPerSec > 0.0)
	{
		point.speedMmPerSec = params.speedMmPerSec;
	}
	traj.points.insert(traj.points.begin() + static_cast<std::ptrdiff_t>(anchor), point);
}

void insertRetractPoint(
	UnifiedTrajectory& traj,
	const RetractParams& params,
	const OpScope& scope,
	const RobotProgram* program)
{
	if (traj.points.empty() || !matchesSegmentSelection(params.segmentSelectMode, params.segmentFrom, params.segmentTo))
	{
		return;
	}
	const std::vector<std::size_t> indices =
		trajectory_algo::resolveScopedPointIndices(traj, scope, program);
	if (indices.empty())
	{
		return;
	}
	const std::size_t anchor = indices.back();
	UnifiedTrajectoryPoint point = traj.points[anchor];
	const Eigen::Vector3d dir = directionByMode(traj, anchor, params.directionMode);
	point.poseMm.x += dir.x() * params.distanceMm;
	point.poseMm.y += dir.y() * params.distanceMm;
	point.poseMm.z += dir.z() * params.distanceMm;
	point.sourceInstructionId.clear();
	if (params.overrideSpeedEnabled && params.speedMmPerSec > 0.0)
	{
		point.speedMmPerSec = params.speedMmPerSec;
	}
	traj.points.insert(traj.points.begin() + static_cast<std::ptrdiff_t>(anchor + 1U), point);
}

void applyAxisReverse(UnifiedTrajectoryPoint& point, const int mirrorAxis)
{
	engine::RigidTransform tf = rigidFromPoint(point);
	Eigen::Matrix3d rot = tf.rotation().toRotationMatrix();
	Eigen::Vector3d axes[3] = { rot.col(0), rot.col(1), rot.col(2) };
	const int reversedAxis = std::max(0, std::min(2, mirrorAxis));
	const int keptAxis = (reversedAxis + 1) % 3;
	const int rebuiltAxis = 3 - reversedAxis - keptAxis;
	axes[reversedAxis] = -axes[reversedAxis];
	axes[rebuiltAxis] = axes[reversedAxis].cross(axes[keptAxis]);
	if (axes[rebuiltAxis].norm() < 1e-9)
	{
		return;
	}
	axes[rebuiltAxis].normalize();
	axes[keptAxis] = axes[rebuiltAxis].cross(axes[reversedAxis]);
	if (axes[keptAxis].norm() < 1e-9)
	{
		return;
	}
	axes[reversedAxis].normalize();
	axes[keptAxis].normalize();
	Eigen::Matrix3d updatedRot;
	updatedRot.col(0) = axes[0];
	updatedRot.col(1) = axes[1];
	updatedRot.col(2) = axes[2];
	pointFromRigid(
		engine::RigidTransform::fromTranslationQuat(
			tf.translationMm(),
			Eigen::Quaterniond(updatedRot)),
		point);
}
} // namespace

bool unifiedTrajectoryFromRaw(const RawTrajectory& raw, UnifiedTrajectory& out, std::string* errMsg)
{
	(void)errMsg;
	out.points.clear();
	out.ctx = raw.ctx;
	out.sourceFeatureJson = raw.sourceFeatureJson;
	out.points.reserve(raw.points.size());
	for (const TrajectoryPoint& point : raw.points)
	{
		UnifiedTrajectoryPoint p{};
		p.poseMm = point.poseMm;
		p.eulerDeg = point.eulerDeg;
		p.blendRadiusMm = point.blendRadiusMm;
		p.speedMmPerSec = point.speedMmPerSec;
		p.reachable = point.reachable;
		out.points.push_back(p);
	}
	return true;
}

bool unifiedTrajectoryFromProgram(const RobotProgram& program, UnifiedTrajectory& out, std::string* errMsg)
{
	out.points.clear();
	std::vector<std::shared_ptr<Base>> flat;
	flattenInstructionsRecursive(program.steps, flat);
	for (const std::shared_ptr<Base>& base : flat)
	{
		if (!base)
		{
			continue;
		}
		UnifiedTrajectoryPoint p{};
		if (!unifiedPointFromMotionInstruction(*base, p))
		{
			continue;
		}
		out.points.push_back(std::move(p));
	}
	if (out.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "no motion waypoints in active program";
		}
		return false;
	}
	return true;
}

bool unifiedTrajectoryFromPathPlanOutput(
	const RobotProgram& program,
	const std::string& pathPlanInstructionId,
	UnifiedTrajectory& out,
	std::string* errMsg)
{
	out.points.clear();
	const InstructionGroup* outputGroup = nullptr;
	for (const InstructionGroup& group : program.groups)
	{
		if (group.role == InstructionGroupRole::PathPlanOutput
			&& group.pathPlanInstructionId == pathPlanInstructionId)
		{
			outputGroup = &group;
			break;
		}
	}
	if (!outputGroup || outputGroup->memberInstructionIds.empty())
	{
		if (errMsg)
		{
			*errMsg = "path plan output group empty";
		}
		return false;
	}
	std::vector<std::shared_ptr<Base>> flat;
	flattenInstructionsRecursive(program.steps, flat);
	std::unordered_map<std::string, std::shared_ptr<Base>> byId;
	byId.reserve(flat.size());
	for (const std::shared_ptr<Base>& base : flat)
	{
		if (base)
		{
			byId.emplace(base->id(), base);
		}
	}
	RobotProgramCatalog catalog;
	const std::vector<std::string> motionIds =
		catalog.expandToMotionWaypointIds(program, outputGroup->memberInstructionIds);
	for (const std::string& id : motionIds)
	{
		const auto it = byId.find(id);
		if (it == byId.end() || !it->second)
		{
			continue;
		}
		UnifiedTrajectoryPoint p{};
		if (!unifiedPointFromMotionInstruction(*it->second, p))
		{
			continue;
		}
		out.points.push_back(std::move(p));
	}
	if (out.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "no motion waypoints in path plan output";
		}
		return false;
	}
	return true;
}

bool unifiedTrajectoryToProgram(
	const UnifiedTrajectory& traj,
	RobotProgram& program,
	std::string* errMsg,
	const bool skipUnreachable)
{
	if (traj.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty unified trajectory";
		}
		return false;
	}
	program.steps.clear();
	program.groups.clear();
	std::vector<std::string> memberIds;
	memberIds.reserve(traj.points.size());
	int idx = 0;
	for (const UnifiedTrajectoryPoint& p : traj.points)
	{
		if (skipUnreachable && !p.reachable)
		{
			continue;
		}
		auto ins = std::make_shared<LineInstruction>();
		ins->setName("P" + std::to_string(++idx));
		const engine::RigidTransform target = rigidFromPoint(p);
		writeTargetTransformToInstruction(*ins, target);
		ins->setBlendRadius(p.blendRadiusMm);
		if (p.speedMmPerSec > 0.0)
		{
			ins->setSpeed(p.speedMmPerSec);
		}
		memberIds.push_back(ins->id());
		program.steps.push_back(ins);
	}
	if (memberIds.empty())
	{
		if (errMsg)
		{
			*errMsg = "no reachable points";
		}
		return false;
	}
	InstructionGroup group;
	group.id = makeGroupId();
	group.name = "UnifiedTrajectory";
	group.memberInstructionIds = std::move(memberIds);
	program.groups.push_back(std::move(group));
	return true;
}

bool unifiedTrajectoryMergeIntoProgram(
	const UnifiedTrajectory& traj,
	RobotProgram& program,
	const std::string& pathPlanInstructionId,
	std::string* errMsg,
	std::string* outOutputGroupId)
{
	if (pathPlanInstructionId.empty())
	{
		return unifiedTrajectoryToProgram(traj, program, errMsg);
	}
	std::unordered_set<std::string> staleMotionIds;
	for (auto it = program.groups.begin(); it != program.groups.end();)
	{
		if (it->role == InstructionGroupRole::PathPlanOutput
			&& it->pathPlanInstructionId == pathPlanInstructionId)
		{
			for (const std::string& id : it->memberInstructionIds)
			{
				staleMotionIds.insert(id);
			}
			it = program.groups.erase(it);
		}
		else
		{
			++it;
		}
	}
	program.steps.erase(
		std::remove_if(
			program.steps.begin(),
			program.steps.end(),
			[&staleMotionIds](const std::shared_ptr<Base>& ins) {
				return ins && staleMotionIds.count(ins->id()) != 0;
			}),
		program.steps.end());
	RobotProgram motionPart;
	if (!unifiedTrajectoryToProgram(traj, motionPart, errMsg))
	{
		return false;
	}
	for (std::shared_ptr<Base>& ins : motionPart.steps)
	{
		program.steps.push_back(std::move(ins));
	}
	if (!motionPart.groups.empty())
	{
		InstructionGroup group = std::move(motionPart.groups.back());
		group.role = InstructionGroupRole::PathPlanOutput;
		group.pathPlanInstructionId = pathPlanInstructionId;
		program.groups.push_back(std::move(group));
		if (outOutputGroupId)
		{
			*outOutputGroupId = program.groups.back().id;
		}
	}
	return true;
}

bool unifiedTrajectoryToRaw(const UnifiedTrajectory& traj, RawTrajectory& raw, std::string* errMsg)
{
	if (traj.points.empty())
	{
		if (errMsg)
		{
			*errMsg = "empty unified trajectory";
		}
		return false;
	}
	raw.points.clear();
	raw.ctx = traj.ctx;
	raw.sourceFeatureJson = traj.sourceFeatureJson;
	raw.points.reserve(traj.points.size());
	for (const UnifiedTrajectoryPoint& p : traj.points)
	{
		TrajectoryPoint tp{};
		tp.poseMm = p.poseMm;
		tp.eulerDeg = p.eulerDeg;
		tp.blendRadiusMm = p.blendRadiusMm;
		tp.speedMmPerSec = p.speedMmPerSec;
		tp.reachable = p.reachable;
		raw.points.push_back(tp);
	}
	return true;
}

bool pointMatchesScope(
	const UnifiedTrajectoryPoint& point,
	const OpScope& scope,
	const RobotProgram* program)
{
	if (!program)
	{
		return true;
	}
	RobotProgramCatalog catalog;
	const std::vector<std::string> ids = catalog.resolveOpScopeInstructionIds(scope, *program);
	if (ids.empty())
	{
		return true;
	}
	return std::find(ids.begin(), ids.end(), point.sourceInstructionId) != ids.end();
}

bool applyUnifiedTrajectoryOp(
	const TrajectoryOpDescriptor& op,
	UnifiedTrajectory& traj,
	std::string* errMsg,
	const RobotProgram* program)
{
	(void)errMsg;
	if (traj.points.empty())
	{
		return false;
	}
	if (op.kind == TrajectoryOpKind::Translate || op.kind == TrajectoryOpKind::Rotate)
	{
		std::vector<std::size_t> scoped;
		for (std::size_t i = 0; i < traj.points.size(); ++i)
		{
			if (pointMatchesScope(traj.points[i], op.scope, program))
			{
				scoped.push_back(i);
			}
		}
		if (scoped.empty())
		{
			return true;
		}
		const std::size_t scopeCount = scoped.size();
		for (std::size_t j = 0; j < scopeCount; ++j)
		{
			const std::size_t i = scoped[j];
			const double t = scopeCount <= 1
				? 0.0
				: static_cast<double>(j) / static_cast<double>(scopeCount - 1);
			const TrajectoryOpDescriptor current = interpolateDescriptor(op, t);
			engine::RigidTransform tf = rigidFromPoint(traj.points[i]);
			if (current.kind == TrajectoryOpKind::Translate)
			{
				const engine::RigidTransform delta = trajectory_algo::rigidDeltaFromTranslate(current.translate);
				tf = trajectory_algo::applyTransformDelta(tf, delta, current.translate.frame);
			}
			else
			{
				const engine::RigidTransform delta = trajectory_algo::rigidDeltaFromRotate(current.rotate);
				tf = trajectory_algo::applyTransformDelta(tf, delta, current.rotate.frame);
			}
			pointFromRigid(tf, traj.points[i]);
		}
		return true;
	}
	if (op.kind == TrajectoryOpKind::Mirror)
	{
		for (UnifiedTrajectoryPoint& point : traj.points)
		{
			if (!pointMatchesScope(point, op.scope, program))
			{
				continue;
			}
			applyAxisReverse(point, op.mirrorAxis);
		}
		return true;
	}
	if (op.kind == TrajectoryOpKind::Reorder)
	{
		if (traj.points.empty())
		{
			return false;
		}
		Vec3 refEuler = traj.points.front().eulerDeg;
		for (const UnifiedTrajectoryPoint& point : traj.points)
		{
			if (pointMatchesScope(point, op.scope, program))
			{
				refEuler = point.eulerDeg;
				break;
			}
		}
		for (UnifiedTrajectoryPoint& point : traj.points)
		{
			if (!pointMatchesScope(point, op.scope, program))
			{
				continue;
			}
			point.eulerDeg = refEuler;
		}
		return true;
	}
	if (op.kind == TrajectoryOpKind::Approach)
	{
		insertApproachPoint(traj, op.approach, op.scope, program);
		return true;
	}
	if (op.kind == TrajectoryOpKind::Retract)
	{
		insertRetractPoint(traj, op.retract, op.scope, program);
		return true;
	}
	return true;
}

bool applyUnifiedTrajectoryPipeline(
	const std::vector<TrajectoryOpDescriptor>& ops,
	UnifiedTrajectory& traj,
	std::string* errMsg)
{
	for (const TrajectoryOpDescriptor& op : ops)
	{
		if (!applyUnifiedTrajectoryOp(op, traj, errMsg))
		{
			return false;
		}
	}
	return true;
}

} // namespace RobotInstruction

