#ifndef KINEMATICCORE_KINEMATICGRAPH_H
#define KINEMATICCORE_KINEMATICGRAPH_H

#include "KinematicJoint.h"
#include "KinematicLink.h"
#include "kinematic_core_global.h"

#include <string>
#include <vector>

namespace kinematic_core
{
class KINEMATIC_CORE_API KinematicGraph
{
public:
	std::vector<KinematicLink> links;
	std::vector<KinematicJoint> joints;
	int rootLinkIdx = 0;

	int dofCount() const;
	bool validateTree(std::string* errMsg = nullptr) const;
	int linkIndexById(const std::string& id) const;
};

} // namespace kinematic_core

#endif // KINEMATICCORE_KINEMATICGRAPH_H
