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
inline constexpr const char* kExtContextViaTransformQuatCsv = "context.viaTransformQuatCsv";
inline constexpr const char* kExtContextViaTransformTransMmCsv = "context.viaTransformTransMmCsv";

/// FK 目标写入扩展并同步 pose/euler 显示
ROBOT_SCENE_API void writeTargetTransformToInstruction(Base& cmd, const engine::RigidTransform& targetInBase);

/// 读基座下目标：优先 quat+trans 扩展，否则 pose/euler
ROBOT_SCENE_API bool readTargetTransformFromInstruction(const Base& cmd, engine::RigidTransform& outTargetInBase);

/// 写入 Via 变换扩展并同步 viaPose/viaEuler
ROBOT_SCENE_API void writeViaTransformToInstruction(Base& cmd, const engine::RigidTransform& viaInBase);

/// 读 Via：优先扩展，否则 viaPose/viaEuler
ROBOT_SCENE_API bool readViaTransformFromInstruction(const Base& cmd, engine::RigidTransform& outViaInBase);

/// REP：相对工作架 TCP
ROBOT_SCENE_API void writeWorkingTcpToInstruction(Base& cmd, const engine::RigidTransform& tcpInWorking);
ROBOT_SCENE_API bool readWorkingTcpFromInstruction(const Base& cmd, engine::RigidTransform& outTcpInWorking);

} // namespace RobotInstruction

#endif // ROBOTSCENE_ROBOTINSTRUCTIONTRANSFORM_H
