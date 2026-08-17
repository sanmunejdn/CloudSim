/// @file CustomDeviceAssemblyCommit.cpp
/// @brief 自定义设备组装提交

#include "CustomDeviceAssemblyCommit.h"

#include "CustomDeviceKinematics.h"
#include "RobotExternalAxes.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"

#include <cstring>
#include <unordered_map>

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
		double gw[16];
		if (!L.geometryBackendId.empty() && geomWorld(backend, L.geometryBackendId, gw))
		{
			RobotExternal::mat4MulColumnMajor16(invW0, gw, L.restInDeviceW0);
		}
		linkStd.push_back(L);
	}

	std::unordered_map<std::string, std::string> geomByLink;
	for (const CustomDeviceLink& L : linkStd)
	{
		geomByLink[L.id] = L.geometryBackendId;
	}

	std::vector<CustomDeviceJoint> jointStd;
	jointStd.reserve(joints.size());
	for (const CustomDeviceJoint& src : joints)
	{
		CustomDeviceJoint J = src;
		const std::string pg = geomByLink[J.parentLinkId];
		const std::string cg = geomByLink[J.childLinkId];
		double pw[16];
		double cw[16];
		if (!pg.empty() && !cg.empty() && geomWorld(backend, pg, pw) && geomWorld(backend, cg, cw))
		{
			double invP[16];
			if (RobotExternal::mat4InvertRigidColumnMajor(pw, invP))
			{
				RobotExternal::mat4MulColumnMajor16(invP, cw, J.parentToChildRest);
			}
		}
		// Frame 中心缓存到 originMm，导出与无 Frame 时兜底
		if (!pg.empty())
		{
			double pwBake[16];
			if (geomWorld(backend, pg, pwBake))
			{
				(void)CustomDeviceKinematics::bakeMotionCenterFrameToOriginMm(J.motion, pwBake, &backend);
			}
		}
		jointStd.push_back(J);
	}

	device.setLinks(linkStd);
	device.setJoints(jointStd);
	std::vector<double> homes;
	homes.reserve(jointStd.size());
	for (const CustomDeviceJoint& J : jointStd)
	{
		homes.push_back(J.motion.home);
	}
	device.setQValues(homes);
	return CustomDeviceKinematics::applyQ(device, &backend, sink);
}
} // namespace CustomDeviceAssemblyCommit
