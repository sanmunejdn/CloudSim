#ifndef TRAJECTORYALGORITHMBUILTINS_UNIFIEDTRAJECTORYSEMANTICMATH_H
#define TRAJECTORYALGORITHMBUILTINS_UNIFIEDTRAJECTORYSEMANTICMATH_H

/// @file UnifiedTrajectorySemanticMath.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief UnifiedTrajectorySemanticMath 接口

// 位姿语义变换与进退刀插入，供 Translate/Mirror/Approach 等原子块复用
#include "TrajectoryPipelineTypes.h"
#include "UnifiedTrajectory.h"

#include <RigidTransform.h>

namespace trajectory_algo
{
engine::RigidTransform rigidFromPoint(const RobotInstruction::UnifiedTrajectoryPoint& point);
void pointFromRigid(const engine::RigidTransform& tf, RobotInstruction::UnifiedTrajectoryPoint& point);

bool applyTranslateRotateInScope(const RobotInstruction::TrajectoryOpDescriptor& op,
								 RobotInstruction::UnifiedTrajectory& traj,
								 const RobotInstruction::RobotProgram* program);

bool applyMirrorInScope(const RobotInstruction::TrajectoryOpDescriptor& op, RobotInstruction::UnifiedTrajectory& traj,
						const RobotInstruction::RobotProgram* program);

bool applyReorderInScope(const RobotInstruction::TrajectoryOpDescriptor& op, RobotInstruction::UnifiedTrajectory& traj,
						 const RobotInstruction::RobotProgram* program);

void insertApproachInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::ApproachParams& params,
						   const RobotInstruction::OpScope& scope, const RobotInstruction::RobotProgram* program);

void insertRetractInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::RetractParams& params,
						  const RobotInstruction::OpScope& scope, const RobotInstruction::RobotProgram* program);

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_UNIFIEDTRAJECTORYSEMANTICMATH_H
