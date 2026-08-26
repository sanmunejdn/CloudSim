/// @file CustomDeviceAssemblyCommit.cpp

/// @brief 自定义设备组装提交



#include "CustomDeviceAssemblyCommit.h"



#include "CustomDeviceKinematics.h"

#include "CustomDeviceGraphBuilder.h"
#include "CustomDeviceMat4Layout.h"
#include "RobotExternalAxes.h"



#include "BackendDataBase.h"

#include "BackendDataManager.h"

#include "BackendFollowMath.h"

#include "FollowAttachmentComponent.h"

#include "Mat4Ops.h"



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

	const BackendMat4 wm = data->worldMatrix();

	for (int i = 0; i < 16; ++i)

	{

		out[i] = wm.v[i];

	}

	return true;

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

				const BackendMat4 targetW = target->worldMatrix();

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



	std::vector<CustomDeviceLink> linkStd;

	linkStd.reserve(links.size());

	for (const CustomDeviceLink& src : links)

	{

		CustomDeviceLink L = src;

		if (!L.geometryBackendId.empty())

		{

			double gw[16];

			if (resolveGeometryWorldForCommit(backend, L.geometryBackendId, gw))

			{

				double w0Kc[16];

				double invW0Kc[16];

				double gwKc[16];

				double restKc[16];

				CustomDeviceMat4Layout::osgBackendToKinematicCore(w0, w0Kc);

				if (!CustomDeviceMat4Layout::kinematicCoreInvertRigid(w0Kc, invW0Kc))

				{

					kinematic_core::mat4IdentityColumnMajor(invW0Kc);

				}

				CustomDeviceMat4Layout::osgBackendToKinematicCore(gw, gwKc);

				kinematic_core::mat4MulColumnMajor16(invW0Kc, gwKc, restKc);

				CustomDeviceMat4Layout::kinematicCoreToOsgBackend(restKc, L.restInDeviceW0);

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

	CustomDeviceGraphBuilder::computeParentToChildRestFromLinkRestPoses(w0, linkStd, jointStd);

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

void refreshLinkRestPosesFromGeometry(CustomDeviceBackendData& device, BackendDataManager& backend)
{
	if (!device.usesLinkJointGraph() || device.links().empty())
	{
		return;
	}

	device.captureBaseWorldW0FromCurrentWorld();
	double w0[16];
	for (int i = 0; i < 16; ++i)
	{
		w0[i] = device.baseWorldW0().v[i];
	}

	std::vector<CustomDeviceLink> links = device.links();
	for (CustomDeviceLink& L : links)
	{
		if (L.geometryBackendId.empty())
		{
			continue;
		}
		double gw[16];
		if (!resolveGeometryWorldForCommit(backend, L.geometryBackendId, gw))
		{
			continue;
		}
		double w0Kc[16];
		double invW0Kc[16];
		double gwKc[16];
		double restKc[16];
		CustomDeviceMat4Layout::osgBackendToKinematicCore(w0, w0Kc);
		if (!CustomDeviceMat4Layout::kinematicCoreInvertRigid(w0Kc, invW0Kc))
		{
			kinematic_core::mat4IdentityColumnMajor(invW0Kc);
		}
		CustomDeviceMat4Layout::osgBackendToKinematicCore(gw, gwKc);
		kinematic_core::mat4MulColumnMajor16(invW0Kc, gwKc, restKc);
		CustomDeviceMat4Layout::kinematicCoreToOsgBackend(restKc, L.restInDeviceW0);
	}

	std::vector<CustomDeviceJoint> joints = device.joints();
	CustomDeviceGraphBuilder::computeParentToChildRestFromLinkRestPoses(w0, links, joints);
	device.setLinks(links);
	device.setJoints(joints);
}

} // namespace CustomDeviceAssemblyCommit

