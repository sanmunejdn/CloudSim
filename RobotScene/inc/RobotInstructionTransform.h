#pragma once

#include "robot_scene_global.h"
#include "RobotInstructionModel.h"

#include <RigidTransform.h>

namespace RobotInstruction
{

inline constexpr const char* kExtContextTargetTransformQuatCsv = "context.targetTransformQuatCsv";
inline constexpr const char* kExtContextTargetTransformTransMmCsv = "context.targetTransformTransMmCsv";

/// Write canonical FK transform to instruction extensions and sync pose/euler display fields.
ROBOT_SCENE_API void writeTargetTransformToInstruction(Base& cmd, const engine::RigidTransform& targetInBase);

/// Read target in base: prefers quat+trans extensions, else pose/euler.
ROBOT_SCENE_API bool readTargetTransformFromInstruction(const Base& cmd, engine::RigidTransform& outTargetInBase);

} // namespace RobotInstruction
