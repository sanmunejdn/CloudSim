#pragma once

#include "RobotInstructionModel.h"
#include "robot_scene_global.h"

#include <memory>
#include <string>
#include <vector>

namespace RobotInstruction
{

/// Depth-first collection of motion instructions for planning (order matches plan result vector).
ROBOT_SCENE_API std::vector<const Base*> collectMotionInstructions(
	const std::vector<std::shared_ptr<Base>>& program);

ROBOT_SCENE_API void collectMotionInstructionsRecursive(
	const std::vector<std::shared_ptr<Base>>& steps,
	std::vector<const Base*>& out);

/// Depth-first flatten of all instructions (for UI display / legacy adapters).
ROBOT_SCENE_API void flattenInstructionsRecursive(
	const std::vector<std::shared_ptr<Base>>& steps,
	std::vector<std::shared_ptr<Base>>& out);

/// Extension / JSON key for 1-based motion waypoint index (P1, P2, ...).
inline constexpr const char* kMotionPointIndexKey = "motion.pointIndex";

ROBOT_SCENE_API bool isMotionWaypointType(Type t);
ROBOT_SCENE_API int motionPointIndex(const Base& ins);
ROBOT_SCENE_API void setMotionPointIndex(Base& ins, int oneBasedIndex);
ROBOT_SCENE_API std::string formatMotionPointName(int oneBasedIndex);

/// Reassign P1..Pn in program traversal order (matches collectMotionInstructionsRecursive).
ROBOT_SCENE_API void renumberMotionPointIndices(std::vector<std::shared_ptr<Base>>& program);

/// Tree / log summary for PTP and LINE (includes Pn and point ordinal when indexed).
ROBOT_SCENE_API std::string formatMotionWaypointSummary(const Base& ins, bool chinese);

} // namespace RobotInstruction
