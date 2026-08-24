/// @file CustomDeviceAssemblyCommit.cpp

/// @brief 自定义设备组装提交



#include "CustomDeviceAssemblyCommit.h"



#include "CustomDeviceKinematics.h"

#include "RobotExternalAxes.h"



#include "BackendDataBase.h"

#include "BackendDataManager.h"

#include "BackendFollowMath.h"

#include "FollowAttachmentComponent.h"



#include <array>
#include <cmath>
#include <cstring>
#include <queue>

#include <unordered_map>

#include <unordered_set>



namespace CustomDeviceAssemblyCommit

{

namespace

{

bool geomWorld(BackendDataManager& backend, const std::string& gid, double out[16])

{

	const auto data = backend.getData(gid);

	if (!data)

	{

		return false;

	}

	const BackendMat4 wm = data->worldMatrix(&backend);

	for (int i = 0; i < 16; ++i)

	{

		out[i] = wm.v[i];

	}

	return true;

}



bool restTranslationNonZero(const double rest[16])

{

	return std::abs(rest[3]) > 1e-6 || std::abs(rest[7]) > 1e-6 || std::abs(rest[11]) > 1e-6;

}



bool resolveGeometryWorldForCommit(BackendDataManager& backend, const std::string& gid, double outGw[16])

{

	const auto data = backend.getData(gid);

	if (!data || !data->hasPoseProperty())

	{

		return false;

	}

	if (const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(

			data->getComponent(FollowAttachmentComponent::typeKeyStatic())))

	{

		if (follow->enabled() && !follow->targetBackendId().empty())

		{

			const auto target = backend.getData(follow->targetBackendId());

			if (target && target->hasPoseProperty())

			{

				const BackendMat4 targetW = target->worldMatrix(&backend);

				const BackendMat4 localW =

					backend_world_mat_from_pose(follow->localPosition(), follow->localEulerDeg());

				BackendMat4 world{};

				if (backend_mat4_multiply(targetW, localW, world))

				{

					for (int i = 0; i < 16; ++i)

					{

						outGw[i] = world.v[i];

					}

					return true;

				}

			}

		}

	}

	return geomWorld(backend, gid, outGw);

}



void computeParentToChildRestFromLinkRestPoses(const double w0[16], const std::vector<CustomDeviceLink>& links,

											   std::vector<CustomDeviceJoint>& joints)

{

	if (links.empty() || joints.empty())

	{

		return;

	}

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

		RobotExternal::mat4MulColumnMajor16(w0, links[linkIdx.at(fixedId)].restInDeviceW0, wf.data());

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

			double childTarget[16];

			RobotExternal::mat4MulColumnMajor16(w0, links[childIt->second].restInDeviceW0, childTarget);

			double invParent[16];

			if (!RobotExternal::mat4InvertRigidColumnMajor(linkWorldQ0[parentId].data(), invParent))

			{

				continue;

			}

			RobotExternal::mat4MulColumnMajor16(invParent, childTarget, J.parentToChildRest);

			std::array<double, 16> childW{};

			RobotExternal::mat4MulColumnMajor16(linkWorldQ0[parentId].data(), J.parentToChildRest, childW.data());

			linkWorldQ0[J.childLinkId] = childW;

			if (!visited.count(J.childLinkId))

			{

				visited.insert(J.childLinkId);

				bfs.push(J.childLinkId);

			}

		}

	}

}

} // namespace



bool commitGraph(CustomDeviceBackendData& device, const std::vector<CustomDeviceLink>& links,

				 const std::vector<CustomDeviceJoint>& joints, BackendDataManager& backend,

				 IRobotBackendPoseSink* sink)

{

	if (links.empty() || joints.empty())

	{

		return false;

	}



	device.captureBaseWorldW0FromCurrentWorld();

	double w0[16];

	for (int i = 0; i < 16; ++i)

	{

		w0[i] = device.baseWorldW0().v[i];

	}

	double invW0[16];

	if (!RobotExternal::mat4InvertRigidColumnMajor(w0, invW0))

	{

		std::memset(invW0, 0, sizeof(invW0));

		invW0[0] = invW0[5] = invW0[10] = invW0[15] = 1.0;

	}



	std::vector<CustomDeviceLink> linkStd;

	linkStd.reserve(links.size());

	for (const CustomDeviceLink& src : links)

	{

		CustomDeviceLink L = src;

		if (!restTranslationNonZero(L.restInDeviceW0) && !L.geometryBackendId.empty())

		{

			double gw[16];

			if (resolveGeometryWorldForCommit(backend, L.geometryBackendId, gw))

			{

				RobotExternal::mat4MulColumnMajor16(invW0, gw, L.restInDeviceW0);

			}

		}

		linkStd.push_back(L);

	}



	std::vector<CustomDeviceJoint> jointStd;

	jointStd.reserve(joints.size());

	for (const CustomDeviceJoint& src : joints)

	{

		jointStd.push_back(src);

	}

	computeParentToChildRestFromLinkRestPoses(w0, linkStd, jointStd);



	device.setLinks(linkStd);

	device.setJoints(jointStd);

	std::vector<double> homes;

	homes.reserve(jointStd.size());

	for (const CustomDeviceJoint& J : jointStd)

	{

		homes.push_back(J.motion.home);

	}

	device.setQValues(homes);

	// 先 FK 到 home，再按父连杆 FK 系烘焙旋转中心

	if (!CustomDeviceKinematics::applyQ(device, &backend, sink))

	{

		return false;

	}

	CustomDeviceKinematics::rebakeRotateJointOriginsFromFrames(device, &backend);

	return CustomDeviceKinematics::applyQ(device, &backend, sink);

}

} // namespace CustomDeviceAssemblyCommit

