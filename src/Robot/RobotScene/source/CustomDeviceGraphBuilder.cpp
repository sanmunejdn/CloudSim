#include "CustomDeviceGraphBuilder.h"

#include "CustomDeviceMat4Layout.h"
#include "JointMotionAdapters.h"

#include "Mat4Ops.h"

#include <cstring>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace CustomDeviceGraphBuilder
{
void computeParentToChildRestFromLinkRestPoses(const double deviceW0Osg[16],
											   const std::vector<CustomDeviceLink>& links,
											   std::vector<CustomDeviceJoint>& joints)
{
	if (links.empty() || joints.empty())
	{
		return;
	}

	double w0Kc[16];
	CustomDeviceMat4Layout::osgBackendToKinematicCore(deviceW0Osg, w0Kc);

	std::unordered_map<std::string, size_t> linkIdx;
	for (size_t i = 0; i < links.size(); ++i)
	{
		linkIdx[links[i].id] = i;
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

	std::unordered_map<std::string, std::array<double, 16>> linkWorldQ0;
	{
		std::array<double, 16> wf{};
		double rootRestKc[16];
		CustomDeviceMat4Layout::osgBackendToKinematicCore(links[linkIdx.at(fixedId)].restInDeviceW0, rootRestKc);
		kinematic_core::mat4MulColumnMajor16(w0Kc, rootRestKc, wf.data());
		linkWorldQ0[fixedId] = wf;
	}

	std::unordered_map<std::string, std::vector<size_t>> jointsByParent;
	for (size_t ji = 0; ji < joints.size(); ++ji)
	{
		jointsByParent[joints[ji].parentLinkId].push_back(ji);
	}

	std::queue<std::string> bfs;
	std::unordered_set<std::string> visited;
	bfs.push(fixedId);
	visited.insert(fixedId);

	while (!bfs.empty())
	{
		const std::string parentId = bfs.front();
		bfs.pop();
		const auto jointIt = jointsByParent.find(parentId);
		if (jointIt == jointsByParent.end() || !linkWorldQ0.count(parentId))
		{
			continue;
		}

		for (const size_t ji : jointIt->second)
		{
			CustomDeviceJoint& J = joints[ji];
			const auto childIt = linkIdx.find(J.childLinkId);
			if (childIt == linkIdx.end())
			{
				continue;
			}

			double childTargetKc[16];
			double childRestKc[16];
			CustomDeviceMat4Layout::osgBackendToKinematicCore(links[childIt->second].restInDeviceW0, childRestKc);
			kinematic_core::mat4MulColumnMajor16(w0Kc, childRestKc, childTargetKc);

			double invParentKc[16];
			if (!CustomDeviceMat4Layout::kinematicCoreInvertRigid(linkWorldQ0[parentId].data(), invParentKc))
			{
				continue;
			}

			double restKc[16];
			kinematic_core::mat4MulColumnMajor16(invParentKc, childTargetKc, restKc);
			CustomDeviceMat4Layout::kinematicCoreToOsgBackend(restKc, J.parentToChildRest);

			std::array<double, 16> childW{};
			kinematic_core::mat4MulColumnMajor16(linkWorldQ0[parentId].data(), restKc, childW.data());
			linkWorldQ0[J.childLinkId] = childW;

			if (!visited.count(J.childLinkId))
			{
				visited.insert(J.childLinkId);
				bfs.push(J.childLinkId);
			}
		}
	}
}

bool buildGraph(const CustomDeviceBackendData& device, kinematic_core::KinematicGraph& outGraph, int& outRootLinkIdx)
{
	outGraph.links.clear();
	outGraph.joints.clear();
	outRootLinkIdx = 0;

	const std::vector<CustomDeviceLink>& links = device.links();
	std::vector<CustomDeviceJoint> joints = device.joints();
	if (links.empty() || joints.empty())
	{
		return false;
	}

	double w0Osg[16];
	for (int i = 0; i < 16; ++i)
	{
		w0Osg[i] = device.baseWorldW0().v[i];
	}
	computeParentToChildRestFromLinkRestPoses(w0Osg, links, joints);

	for (const CustomDeviceLink& L : links)
	{
		kinematic_core::KinematicLink kl;
		kl.id = L.id;
		kl.payloadKey = L.geometryBackendId;
		kl.fixed = L.fixed;
		CustomDeviceMat4Layout::osgBackendToKinematicCore(L.restInDeviceW0, kl.restInBase);
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
		CustomDeviceMat4Layout::osgBackendToKinematicCore(J.parentToChildRest, kj.parentToChildRest);
		kj.qIndex = static_cast<int>(ji);
		outGraph.joints.push_back(kj);
	}

	std::string err;
	return outGraph.validateTree(&err);
}

} // namespace CustomDeviceGraphBuilder
