#ifndef ROBOTSCENE_CUSTOMDEVICEGRAPHBUILDER_H
#define ROBOTSCENE_CUSTOMDEVICEGRAPHBUILDER_H

#include "CustomDeviceBackendData.h"
#include "robot_scene_global.h"

#include "KinematicGraph.h"

namespace CustomDeviceGraphBuilder
{
ROBOT_SCENE_API bool buildGraph(const CustomDeviceBackendData& device, kinematic_core::KinematicGraph& outGraph,
								int& outRootLinkIdx);
}

#endif // ROBOTSCENE_CUSTOMDEVICEGRAPHBUILDER_H
