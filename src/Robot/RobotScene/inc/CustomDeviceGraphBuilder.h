#ifndef ROBOTSCENE_CUSTOMDEVICEGRAPHBUILDER_H
#define ROBOTSCENE_CUSTOMDEVICEGRAPHBUILDER_H

/// @file CustomDeviceGraphBuilder.h
/// @brief 由 Link.restInDeviceW0 沿树 BFS 计算各 Joint 的 parentToChildRest（OSG/Backend 布局）

#include "robot_scene_global.h"

#include "CustomDeviceBackendData.h"
#include "KinematicGraph.h"

namespace CustomDeviceGraphBuilder
{
/// 由 Link.restInDeviceW0 沿树 BFS 计算各 Joint 的 parentToChildRest（OSG/Backend 布局）
ROBOT_SCENE_API void computeParentToChildRestFromLinkRestPoses(const double deviceW0Osg[16],
															   const std::vector<CustomDeviceLink>& links,
															   std::vector<CustomDeviceJoint>& joints);

ROBOT_SCENE_API bool buildGraph(const CustomDeviceBackendData& device, kinematic_core::KinematicGraph& outGraph,
								int& outRootLinkIdx);
} // namespace CustomDeviceGraphBuilder

#endif // ROBOTSCENE_CUSTOMDEVICEGRAPHBUILDER_H
