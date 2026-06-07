// UnifiedTrajectory 路径几何原语，供原子块复用
#pragma once

#include "TrajectoryPipelineTypes.h"
#include "TrajectoryUnifiedScope.h"
#include "UnifiedTrajectory.h"

namespace trajectory_algo
{

void resampleUnifiedTrajectory(RobotInstruction::UnifiedTrajectory& traj, double stepMm);
void offsetAlongNormalUnified(RobotInstruction::UnifiedTrajectory& traj, double offsetMm);
void offsetLateralUnified(RobotInstruction::UnifiedTrajectory& traj, double lateralMm);
void smoothPoseUnified(RobotInstruction::UnifiedTrajectory& traj);
void assignBlendUnified(RobotInstruction::UnifiedTrajectory& traj, double blendRadiusMm);
void assignSpeedUnified(RobotInstruction::UnifiedTrajectory& traj, double speedMmPerSec);
void weaveUnified(
	RobotInstruction::UnifiedTrajectory& traj,
	double amplitudeMm,
	double periodMm);
void reachabilityFilterUnified(RobotInstruction::UnifiedTrajectory& traj);
void externalAxisSearchUnified(RobotInstruction::UnifiedTrajectory& traj);

void resampleUnifiedTrajectoryInScope(
	RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program,
	double stepMm);
void offsetAlongNormalUnifiedInScope(
	RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program,
	double offsetMm);
void offsetLateralUnifiedInScope(
	RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program,
	double lateralMm);
void smoothPoseUnifiedInScope(
	RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program);
void assignBlendUnifiedInScope(
	RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program,
	double blendRadiusMm);
void assignSpeedUnifiedInScope(
	RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program,
	double speedMmPerSec);
void weaveUnifiedInScope(
	RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program,
	double amplitudeMm,
	double periodMm);
void reachabilityFilterUnifiedInScope(
	RobotInstruction::UnifiedTrajectory& traj,
	const RobotInstruction::OpScope& scope,
	const RobotInstruction::RobotProgram* program);

} // namespace trajectory_algo
