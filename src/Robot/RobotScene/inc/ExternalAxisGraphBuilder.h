#ifndef ROBOTSCENE_EXTERNALAXISGRAPHBUILDER_H
#define ROBOTSCENE_EXTERNALAXISGRAPHBUILDER_H

/// @file ExternalAxisGraphBuilder.h
/// @brief 外部轴链：虚拟 link0→link1→…，attachment 由调用方写入 baseWorld

#include "robot_scene_global.h"

#include "KinematicGraph.h"
#include "RobotExternalAxes.h"

namespace ExternalAxisGraphBuilder
{
/// 外部轴链：虚拟 link0→link1→…，attachment 由调用方写入 baseWorld
ROBOT_SCENE_API bool buildChain(const RobotExternal::RobotExternalAxisConfigSet& set,
								RobotExternal::RobotExternalAttachment attachment,
								kinematic_core::KinematicGraph& outGraph, std::vector<int>& outAxisIndicesInSet);
} // namespace ExternalAxisGraphBuilder

#endif // ROBOTSCENE_EXTERNALAXISGRAPHBUILDER_H
