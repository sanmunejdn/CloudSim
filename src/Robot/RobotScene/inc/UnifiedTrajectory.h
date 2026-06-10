#pragma once

#include "RawTrajectory.h"
#include "RobotProgramCatalog.h"
#include "TrajectoryPipelineTypes.h"
#include "robot_scene_global.h"

#include <string>
#include <vector>

namespace RobotInstruction
{

struct ROBOT_SCENE_API UnifiedTrajectoryPoint
{
	Vec3 poseMm{};
	Vec3 eulerDeg{};
	double blendRadiusMm = 0.0;
	double speedMmPerSec = 0.0;
	bool reachable = true;
	std::string sourceInstructionId;
};

struct ROBOT_SCENE_API UnifiedTrajectory
{
	std::vector<UnifiedTrajectoryPoint> points;
	TrajectoryContext ctx{};
	std::string sourceFeatureJson;
};

ROBOT_SCENE_API bool unifiedTrajectoryFromRaw(
	const RawTrajectory& raw,
	UnifiedTrajectory& out,
	std::string* errMsg = nullptr);

ROBOT_SCENE_API bool unifiedTrajectoryFromProgram(
	const RobotProgram& program,
	UnifiedTrajectory& out,
	std::string* errMsg = nullptr);

/// 仅导入指定 PathPlan 的 PathPlanOutput 成员路点（顺序与分组一致）
ROBOT_SCENE_API bool unifiedTrajectoryFromPathPlanOutput(
	const RobotProgram& program,
	const std::string& pathPlanInstructionId,
	UnifiedTrajectory& out,
	std::string* errMsg = nullptr);

ROBOT_SCENE_API bool unifiedTrajectoryToProgram(
	const UnifiedTrajectory& traj,
	RobotProgram& program,
	std::string* errMsg = nullptr,
	bool skipUnreachable = true);

/// Apply 时保留根级 PathPlan/逻辑指令，替换指定 PathPlan 的运动输出与 PathPlanOutput 分组
ROBOT_SCENE_API bool unifiedTrajectoryMergeIntoProgram(
	const UnifiedTrajectory& traj,
	RobotProgram& program,
	const std::string& pathPlanInstructionId,
	std::string* errMsg = nullptr,
	std::string* outOutputGroupId = nullptr);

ROBOT_SCENE_API bool unifiedTrajectoryToRaw(
	const UnifiedTrajectory& traj,
	RawTrajectory& raw,
	std::string* errMsg = nullptr);

} // namespace RobotInstruction

