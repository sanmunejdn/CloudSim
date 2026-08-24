#include "ExternalAxisGraphBuilder.h"

#include "JointMotionAdapters.h"
#include "Mat4Ops.h"

namespace ExternalAxisGraphBuilder
{
bool buildChain(const RobotExternal::RobotExternalAxisConfigSet& set,
				const RobotExternal::RobotExternalAttachment attachment, kinematic_core::KinematicGraph& outGraph,
				std::vector<int>& outAxisIndicesInSet)
{
	outGraph.links.clear();
	outGraph.joints.clear();
	outAxisIndicesInSet.clear();

	kinematic_core::KinematicLink root;
	root.id = "ext_root";
	root.fixed = true;
	outGraph.links.push_back(root);

	int parentIdx = 0;
	int qIndex = 0;
	for (int i = 0; i < static_cast<int>(set.axes.size()); ++i)
	{
		const RobotExternal::RobotExternalAxisConfig& cfg = set.axes[static_cast<size_t>(i)];
		if (!cfg.enabled || cfg.attachment != attachment)
		{
			continue;
		}
		kinematic_core::KinematicLink child;
		child.id = "ext_link_" + std::to_string(i);
		outGraph.links.push_back(child);
		const int childIdx = static_cast<int>(outGraph.links.size()) - 1;

		kinematic_core::KinematicJoint joint;
		joint.parentLinkIdx = parentIdx;
		joint.childLinkIdx = childIdx;
		joint.motion = JointMotionAdapters::fromRobotExternalAxisConfig(cfg);
		joint.qIndex = qIndex++;
		kinematic_core::mat4IdentityColumnMajor(joint.parentToChildRest);
		outGraph.joints.push_back(joint);

		outAxisIndicesInSet.push_back(i);
		parentIdx = childIdx;
	}

	outGraph.rootLinkIdx = 0;
	if (outGraph.joints.empty())
	{
		return false;
	}
	std::string err;
	return outGraph.validateTree(&err);
}

} // namespace ExternalAxisGraphBuilder
