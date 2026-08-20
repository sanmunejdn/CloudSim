#ifndef ROBOTSCENE_ROBOTSCENENONRIGIDTRAJECTORYWARP_H
#define ROBOTSCENE_ROBOTSCENENONRIGIDTRAJECTORYWARP_H

/// @file RobotSceneNonRigidTrajectoryWarp.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief RobotSceneNonRigidTrajectoryWarp 接口

#include "robot_scene_global.h"

#include <INonRigidTrajectoryWarp.h>

namespace RobotInstruction
{
class ROBOT_SCENE_API RobotSceneNonRigidTrajectoryWarp final : public trajectory_algo::INonRigidTrajectoryWarp
{
public:
	bool warp(UnifiedTrajectory& traj, const NonRigidRegistrationParams& params, const OpScope& scope,
			  const RobotProgram* program, std::size_t* missCount, std::string* errMsg) const override;
};

ROBOT_SCENE_API const RobotSceneNonRigidTrajectoryWarp& robotSceneNonRigidTrajectoryWarp();

ROBOT_SCENE_API bool nonRigidWarpUnifiedTrajectory(UnifiedTrajectory& traj, const NonRigidRegistrationParams& params,
												   const OpScope& scope, const RobotProgram* program,
												   std::size_t* outMissCount, std::string* errMsg);

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTSCENENONRIGIDTRAJECTORYWARP_H
