#ifndef ROBOTSCENE_ROBOTINSTRUCTIONTRANSFORM_H
#define ROBOTSCENE_ROBOTINSTRUCTIONTRANSFORM_H

/// @file RobotInstructionTransform.h
/// @brief FK 目标写入扩展并同步 pose/euler 显示

#include "robot_scene_global.h"

#include "RobotInstructionModel.h"

#include <RigidTransform.h>

namespace RobotInstruction
{
inline constexpr const char* kExtContextTargetTransformQuatCsv = "context.targetTransformQuatCsv";
inline constexpr const char* kExtContextTargetTransformTransMmCsv = "context.targetTransformTransMmCsv";

/// FK 目标写入扩展并同步 pose/euler 显示
ROBOT_SCENE_API void writeTargetTransformToInstruction(Base& cmd, const engine::RigidTransform& targetInBase);

/// 读基座下目标：优先 quat+trans 扩展，否则 pose/euler
ROBOT_SCENE_API bool readTargetTransformFromInstruction(const Base& cmd, engine::RigidTransform& outTargetInBase);

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONTRANSFORM_H
