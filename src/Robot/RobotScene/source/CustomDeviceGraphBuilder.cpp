#include "CustomDeviceGraphBuilder.h"

#include "JointMotionAdapters.h"

#include <cstring>

namespace CustomDeviceGraphBuilder
{
bool buildGraph(const CustomDeviceBackendData& device, kinematic_core::KinematicGraph& outGraph, int& outRootLinkIdx)
{
	outGraph.links.clear();
	outGraph.joints.clear();
	outRootLinkIdx = 0;

	const std::vector<CustomDeviceLink>& links = device.links();
	const std::vector<CustomDeviceJoint>& joints = device.joints();
	if (links.empty() || joints.empty())
	{
		return false;
	}

	for (const CustomDeviceLink& L : links)
	{
		kinematic_core::KinematicLink kl;
		kl.id = L.id;
		kl.payloadKey = L.geometryBackendId;
		kl.fixed = L.fixed;
		std::memcpy(kl.restInBase, L.restInDeviceW0, sizeof(kl.restInBase));
		outGraph.links.push_back(std::move(kl));
	}

	std::string fixedId;
	for (const CustomDeviceLink& L : links)
	{
		if (L.fixed)
		{
			fixedId = L.id;
			break;
		}
	}
	if (fixedId.empty())
	{
		fixedId = links.front().id;
	}
	outRootLinkIdx = outGraph.linkIndexById(fixedId);
	outGraph.rootLinkIdx = outRootLinkIdx;
	if (outRootLinkIdx < 0)
	{
		return false;
	}

	for (size_t ji = 0; ji < joints.size(); ++ji)
	{
		const CustomDeviceJoint& J = joints[ji];
		kinematic_core::KinematicJoint kj;
		kj.parentLinkIdx = outGraph.linkIndexById(J.parentLinkId);
		kj.childLinkIdx = outGraph.linkIndexById(J.childLinkId);
		if (kj.parentLinkIdx < 0 || kj.childLinkIdx < 0)
		{
			continue;
		}
		kj.motion = JointMotionAdapters::fromCustomDeviceAxisConfig(J.motion);
		std::memcpy(kj.parentToChildRest, J.parentToChildRest, sizeof(kj.parentToChildRest));
		kj.qIndex = static_cast<int>(ji);
		outGraph.joints.push_back(kj);
	}

	std::string err;
	return outGraph.validateTree(&err);
}

} // namespace CustomDeviceGraphBuilder
