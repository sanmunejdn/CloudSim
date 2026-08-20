#ifndef ROBOTSCENE_TRAJECTORYPATHADAPTERS_H
#define ROBOTSCENE_TRAJECTORYPATHADAPTERS_H

/// @file TrajectoryPathAdapters.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief raw 转 unified 时由调用方注入世界系变换（Session 持有 OSG 上下文）

#include "robot_scene_global.h"

#include "RawTrajectory.h"
#include "RobotInstructionModel.h"
#include "UnifiedTrajectory.h"

#include <functional>
#include <string>

namespace RobotInstruction
{
/// raw 转 unified 时由调用方注入世界系变换（Session 持有 OSG 上下文）
using RawToUnifiedRebuildFn =
	std::function<bool(const RawTrajectory& sourceRaw, UnifiedTrajectory& outUnified, std::string* errMsg)>;

ROBOT_SCENE_API bool ingressUnifiedFromProgram(const RobotProgram& program, UnifiedTrajectory& out,
											   std::string* errMsg = nullptr);

/// pathPlanId 非空时仅 Ingress 该 PathPlan 输出组路点
ROBOT_SCENE_API bool ingressUnifiedForEdit(const RobotProgram& program, const std::string& pathPlanInstructionId,
										   UnifiedTrajectory& out, std::string* errMsg = nullptr);

ROBOT_SCENE_API bool ingressUnifiedFromRaw(const RawTrajectory& sourceRaw, const RawToUnifiedRebuildFn& rebuild,
										   UnifiedTrajectory& out, std::string* errMsg = nullptr);

ROBOT_SCENE_API bool egressUnifiedToProgram(const UnifiedTrajectory& traj, RobotProgram& program,
											std::string* errMsg = nullptr);

ROBOT_SCENE_API bool egressUnifiedMergeIntoProgram(const UnifiedTrajectory& traj, RobotProgram& program,
												   const std::string& pathPlanInstructionId = {},
												   std::string* errMsg = nullptr,
												   std::string* outOutputGroupId = nullptr);

ROBOT_SCENE_API bool egressUnifiedToRaw(const UnifiedTrajectory& traj, RawTrajectory& raw,
										std::string* errMsg = nullptr);

} // namespace RobotInstruction

#endif // ROBOTSCENE_TRAJECTORYPATHADAPTERS_H
