#ifndef GEOMETRYENGINE_TOOLKINEMATICS_H
#define GEOMETRYENGINE_TOOLKINEMATICS_H

/// @file ToolKinematics.h
/// @brief T_base_tool = T_base_flange * T_flange_tool（法兰轴系下工具偏移）

#include "geometry_engine_global.h"

#include "RigidTransform.h"

namespace engine
{
/// T_base_tool = T_base_flange * T_flange_tool（法兰轴系下工具偏移）
GEOMETRY_ENGINE_API RigidTransform toolOriginFromFlange(const RigidTransform& baseFlange,
														const RigidTransform& flangeTool);

/// T_base_flange = T_base_tool * inv(T_flange_tool)
GEOMETRY_ENGINE_API RigidTransform flangeFromToolOrigin(const RigidTransform& baseToolOrigin,
														const RigidTransform& flangeTool);

} // namespace engine

#endif // GEOMETRYENGINE_TOOLKINEMATICS_H
