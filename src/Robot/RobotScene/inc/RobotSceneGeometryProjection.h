#pragma once

#include <TrajectoryOpExecutionContext.h>

#include "robot_scene_global.h"

namespace RobotInstruction
{

class ROBOT_SCENE_API RobotSceneGeometryProjection final : public trajectory_algo::IGeometryProjection
{
public:
	bool project(
		UnifiedTrajectory& traj,
		const ProjectToGeometryParams& params,
		const OpScope& scope,
		const RobotProgram* program,
		std::size_t* missCount,
		std::string* errMsg) const override;
};

ROBOT_SCENE_API const RobotSceneGeometryProjection& robotSceneGeometryProjection();

} // namespace RobotInstruction
