#ifndef KINEMATICCORE_TREEFORWARDKINEMATICS_H
#define KINEMATICCORE_TREEFORWARDKINEMATICS_H

/// @file TreeForwardKinematics.h
/// @brief childWorld = parentWorld * T_motion(q) * parentToChildRest

#include "kinematic_core_global.h"

#include "KinematicGraph.h"

#include <vector>

namespace kinematic_core
{
/// childWorld = parentWorld * T_motion(q) * parentToChildRest
KINEMATIC_CORE_API bool forwardKinematicsTree(const KinematicGraph& graph, const double baseWorld[16], const double* q,
											  std::size_t qCount, double linkWorld[][16]);

KINEMATIC_CORE_API bool forwardKinematicsTree(const KinematicGraph& graph, const double baseWorld[16], const double* q,
											  std::size_t qCount, std::vector<double>& flatLinkWorld16);

} // namespace kinematic_core

#endif // KINEMATICCORE_TREEFORWARDKINEMATICS_H
