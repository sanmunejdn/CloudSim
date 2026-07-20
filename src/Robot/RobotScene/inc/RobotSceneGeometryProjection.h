#ifndef ROBOTSCENE_ROBOTSCENEGEOMETRYPROJECTION_H
#define ROBOTSCENE_ROBOTSCENEGEOMETRYPROJECTION_H

/// @file RobotSceneGeometryProjection.h
/// @brief RobotSceneGeometryProjection 接口

#include "robot_scene_global.h"

#include <TrajectoryOpExecutionContext.h>

namespace RobotInstruction
{
class ROBOT_SCENE_API RobotSceneGeometryProjection final : public trajectory_algo::IGeometryProjection
{
public:
	bool project(UnifiedTrajectory& traj, const ProjectToGeometryParams& params, const OpScope& scope,
				 const RobotProgram* program, std::size_t* missCount, std::string* errMsg) const override;
};

ROBOT_SCENE_API const RobotSceneGeometryProjection& robotSceneGeometryProjection();

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTSCENEGEOMETRYPROJECTION_H
