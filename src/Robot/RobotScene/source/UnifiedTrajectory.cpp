/// @file UnifiedTrajectory.cpp
/// @brief UnifiedTrajectory 实现

#include "UnifiedTrajectory.h"

#include "RobotInstructionFactory.h"
#include "RobotInstructionProgram.h"
#include "RobotInstructionTransform.h"
#include "RobotProgramCatalog.h"

#include <algorithm>
#include <unordered_map>
#include <unordered_set>

#include <RigidTransform.h>
#include <UnifiedTrajectorySemanticMath.h>

namespace RobotInstruction
{
namespace
{
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
	trajectory_algo::pointFromRigid(target, out);
	out.sourceInstructionId = base.id();
	out.blendRadiusMm = base.blendRadius();
	out.speedMmPerSec = base.speed();
	out.reachable = true;
	return true;
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

bool unifiedTrajectoryFromPathPlanOutput(const RobotProgram& program, const std::string& pathPlanInstructionId,
										 UnifiedTrajectory& out, std::string* errMsg)
{
	out.points.clear();
	const InstructionGroup* outputGroup = nullptr;
	for (const InstructionGroup& group : program.groups)
	{
		if (group.role == InstructionGroupRole::PathPlanOutput && group.pathPlanInstructionId == pathPlanInstructionId)
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

bool unifiedTrajectoryToProgram(const UnifiedTrajectory& traj, RobotProgram& program, std::string* errMsg,
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
		const engine::RigidTransform target = trajectory_algo::rigidFromPoint(p);
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

bool unifiedTrajectoryMergeIntoProgram(const UnifiedTrajectory& traj, RobotProgram& program,
									   const std::string& pathPlanInstructionId, std::string* errMsg,
									   std::string* outOutputGroupId)
{
	if (pathPlanInstructionId.empty())
	{
		return unifiedTrajectoryToProgram(traj, program, errMsg);
	}
	std::unordered_set<std::string> staleMotionIds;
	for (auto it = program.groups.begin(); it != program.groups.end();)
	{
		if (it->role == InstructionGroupRole::PathPlanOutput && it->pathPlanInstructionId == pathPlanInstructionId)
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
	program.steps.erase(std::remove_if(program.steps.begin(), program.steps.end(),
									   [&staleMotionIds](const std::shared_ptr<Base>& ins)
									   { return ins && staleMotionIds.count(ins->id()) != 0; }),
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
} // namespace RobotInstruction
