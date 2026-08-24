#ifndef KINEMATICCORE_TREEFORWARDKINEMATICS_H
#define KINEMATICCORE_TREEFORWARDKINEMATICS_H

#include "KinematicGraph.h"
#include "kinematic_core_global.h"

#include <vector>

namespace kinematic_core
{
/// childWorld = parentWorld * T_motion(q) * parentToChildRest
KINEMATIC_CORE_API bool forwardKinematicsTree(const KinematicGraph& graph, const double baseWorld[16],
											  const double* q, std::size_t qCount, double linkWorld[][16]);

KINEMATIC_CORE_API bool forwardKinematicsTree(const KinematicGraph& graph, const double baseWorld[16],
											  const double* q, std::size_t qCount, std::vector<double>& flatLinkWorld16);

} // namespace kinematic_core

#endif // KINEMATICCORE_TREEFORWARDKINEMATICS_H
