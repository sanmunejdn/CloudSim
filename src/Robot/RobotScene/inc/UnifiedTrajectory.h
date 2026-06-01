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
};

ROBOT_SCENE_API bool unifiedTrajectoryFromRaw(
	const RawTrajectory& raw,
	UnifiedTrajectory& out,
	std::string* errMsg = nullptr);

ROBOT_SCENE_API bool unifiedTrajectoryFromProgram(
	const RobotProgram& program,
	UnifiedTrajectory& out,
	std::string* errMsg = nullptr);

ROBOT_SCENE_API bool unifiedTrajectoryToProgram(
	const UnifiedTrajectory& traj,
	RobotProgram& program,
	std::string* errMsg = nullptr);

ROBOT_SCENE_API bool unifiedTrajectoryToRaw(
	const UnifiedTrajectory& traj,
	RawTrajectory& raw,
	std::string* errMsg = nullptr);

ROBOT_SCENE_API bool applyUnifiedTrajectoryOp(
	const TrajectoryOpDescriptor& op,
	UnifiedTrajectory& traj,
	std::string* errMsg = nullptr);

ROBOT_SCENE_API bool applyUnifiedTrajectoryPipeline(
	const std::vector<TrajectoryOpDescriptor>& ops,
	UnifiedTrajectory& traj,
	std::string* errMsg = nullptr);

} // namespace RobotInstruction

