#ifndef ROBOTURDF_URDFGRAPHBUILDER_H
#define ROBOTURDF_URDFGRAPHBUILDER_H

#include "robot_urdf_global.h"

#include "KinematicGraph.h"

#include <QString>

namespace UrdfGraphBuilder
{
ROBOT_URDF_API bool buildGraphFromUrdfFile(const QString& urdfFilePath, kinematic_core::KinematicGraph& outGraph,
										   QString* errorMessage = nullptr);
}

#endif // ROBOTURDF_URDFGRAPHBUILDER_H
