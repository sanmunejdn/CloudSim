#pragma once

#include "RobotInstructionModel.h"
#include "robot_scene_global.h"

#include <memory>
#include <string>
#include <vector>

namespace RobotInstruction
{

/// 深度优先收集运动指令（顺序与规划结果向量一致）
ROBOT_SCENE_API std::vector<const Base*> collectMotionInstructions(
	const std::vector<std::shared_ptr<Base>>& program);

ROBOT_SCENE_API void collectMotionInstructionsRecursive(
	const std::vector<std::shared_ptr<Base>>& steps,
	std::vector<const Base*>& out);

/// 深度优先展平全部指令（UI/旧适配）
ROBOT_SCENE_API void flattenInstructionsRecursive(
	const std::vector<std::shared_ptr<Base>>& steps,
	std::vector<std::shared_ptr<Base>>& out);

/// 运动路点扩展键：1-based 序号 P1、P2…
inline constexpr const char* kMotionPointIndexKey = "motion.pointIndex";

ROBOT_SCENE_API bool isMotionWaypointType(Type t);
ROBOT_SCENE_API int motionPointIndex(const Base& ins);
ROBOT_SCENE_API void setMotionPointIndex(Base& ins, int oneBasedIndex);
ROBOT_SCENE_API std::string formatMotionPointName(int oneBasedIndex);

/// 按遍历顺序重编号 P1..Pn（同 collectMotionInstructionsRecursive）
ROBOT_SCENE_API void renumberMotionPointIndices(std::vector<std::shared_ptr<Base>>& program);

/// PTP/LINE 树/日志摘要（含 Pn 与路点序号）
ROBOT_SCENE_API std::string formatMotionWaypointSummary(const Base& ins, bool chinese);

} // namespace RobotInstruction
