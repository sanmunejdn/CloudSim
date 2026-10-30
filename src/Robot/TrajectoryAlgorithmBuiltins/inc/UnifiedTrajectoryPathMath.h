#ifndef TRAJECTORYALGORITHMBUILTINS_UNIFIEDTRAJECTORYPATHMATH_H
#define TRAJECTORYALGORITHMBUILTINS_UNIFIEDTRAJECTORYPATHMATH_H

/// @file UnifiedTrajectoryPathMath.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief UnifiedTrajectoryPathMath 接口

// UnifiedTrajectory 路径几何原语，供原子块复用
#include "TrajectoryOpExecutionContext.h"
#include "TrajectoryPipelineTypes.h"
#include "TrajectoryUnifiedScope.h"
#include "UnifiedTrajectory.h"

namespace trajectory_algo
{
void resampleUnifiedTrajectory(RobotInstruction::UnifiedTrajectory& traj, double stepMm);
void offsetLateralUnified(RobotInstruction::UnifiedTrajectory& traj, double lateralMm);
void smoothPoseUnified(RobotInstruction::UnifiedTrajectory& traj);
void assignBlendUnified(RobotInstruction::UnifiedTrajectory& traj, double blendRadiusMm);
void assignSpeedUnified(RobotInstruction::UnifiedTrajectory& traj, double speedMmPerSec);
void weaveUnified(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::WeaveParams& params);
/// 委托 ctx.reachabilityProbe；未注入返回 false
bool reachabilityFilterUnified(RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx,
							   bool useOrientation, double residualTolMm, std::string* errMsg);
/// 无配置时为 no-op；有配置时委托 ctx.externalAxisSearch
void externalAxisSearchUnified(RobotInstruction::UnifiedTrajectory& traj, const TrajectoryOpExecutionContext& ctx);

void resampleUnifiedTrajectoryInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
									  const RobotInstruction::RobotProgram* program, double stepMm);
void offsetLateralUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
								 const RobotInstruction::RobotProgram* program, double lateralMm);
void smoothPoseUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							  const RobotInstruction::RobotProgram* program);
void assignBlendUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							   const RobotInstruction::RobotProgram* program, double blendRadiusMm);
void assignSpeedUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							   const RobotInstruction::RobotProgram* program, double speedMmPerSec);
void weaveUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
						 const RobotInstruction::RobotProgram* program, const RobotInstruction::WeaveParams& params);
bool reachabilityFilterUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
									  const RobotInstruction::RobotProgram* program,
									  const TrajectoryOpExecutionContext& ctx, bool useOrientation,
									  double residualTolMm, std::string* errMsg);

void reversePathUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							   const RobotInstruction::RobotProgram* program);
void orientToPathUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
								const RobotInstruction::RobotProgram* program, bool flipToolZ);
void trimPathUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							const RobotInstruction::RobotProgram* program, double startMm, double endMm);
void shiftStartUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							  const RobotInstruction::RobotProgram* program, int startIndex1);
void insertMidpointsUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
								   const RobotInstruction::RobotProgram* program, int passes);
void clampToPlaneUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
								const RobotInstruction::RobotProgram* program, int axis, double valueMm);
void scalePathUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							 const RobotInstruction::RobotProgram* program, double factor);
void speedRampUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							 const RobotInstruction::RobotProgram* program, double startMmPerSec, double endMmPerSec);
void spikeFilterUnifiedInScope(RobotInstruction::UnifiedTrajectory& traj, const RobotInstruction::OpScope& scope,
							   const RobotInstruction::RobotProgram* program, double maxDetourRatio);

} // namespace trajectory_algo

#endif // TRAJECTORYALGORITHMBUILTINS_UNIFIEDTRAJECTORYPATHMATH_H
