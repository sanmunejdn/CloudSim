#pragma once

#include <INonRigidTrajectoryWarp.h>

#include "robot_scene_global.h"

namespace RobotInstruction
{

class ROBOT_SCENE_API RobotSceneNonRigidTrajectoryWarp final
	: public trajectory_algo::INonRigidTrajectoryWarp
{
public:
	bool warp(
		UnifiedTrajectory& traj,
		const NonRigidRegistrationParams& params,
		const OpScope& scope,
		const RobotProgram* program,
		std::size_t* missCount,
		std::string* errMsg) const override;
};

ROBOT_SCENE_API const RobotSceneNonRigidTrajectoryWarp& robotSceneNonRigidTrajectoryWarp();

ROBOT_SCENE_API bool nonRigidWarpUnifiedTrajectory(
	UnifiedTrajectory& traj,
	const NonRigidRegistrationParams& params,
	const OpScope& scope,
	const RobotProgram* program,
	std::size_t* outMissCount,
	std::string* errMsg);

} // namespace RobotInstruction
