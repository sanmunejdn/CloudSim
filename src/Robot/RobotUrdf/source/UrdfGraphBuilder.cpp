/// @file UrdfGraphBuilder.cpp
/// @brief UrdfGraphBuilder 实现

#include "UrdfGraphBuilder.h"

#include "UrdfRobotLoader.h"

namespace UrdfGraphBuilder
{
bool buildGraphFromUrdfFile(const QString& urdfFilePath, kinematic_core::KinematicGraph& outGraph,
							QString* errorMessage)
{
	return UrdfRobotLoader::buildUrdfKinematicGraph(urdfFilePath, outGraph, errorMessage);
}

} // namespace UrdfGraphBuilder
