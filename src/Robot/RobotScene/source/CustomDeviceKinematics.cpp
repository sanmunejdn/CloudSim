/// @file CustomDeviceKinematics.cpp
/// @brief CustomDeviceKinematics 实现（扁平轴 + Link/Joint 树）

#include "CustomDeviceKinematics.h"

#include "CustomDeviceKinematicModel.h"
#include "CustomDeviceMat4Layout.h"
#include "CustomDeviceAssemblyCommit.h"
#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "BackendSpatial.h"
#include "IRobotBackendPoseSink.h"

#include "CoreTypes.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <queue>
#include <string>
#include <unordered_map>
#include <unordered_set>

namespace CustomDeviceKinematics
{
namespace
{
void backendMat4ToArray(const BackendMat4& m, double out[16])
{
	for (int i = 0; i < 16; ++i)
	{
		out[i] = m.v[i];
	}
}

BackendMat4 arrayToBackendMat4(const double in[16])
{
	BackendMat4 m = BackendMat4::identity();
	for (int i = 0; i < 16; ++i)
	{
		m.v[i] = in[i];
	}
	return m;
}

bool computeLinkWorldMatrices(const CustomDeviceBackendData& device, BackendDataManager* mgr,
							  const std::vector<double>& q,
							  std::unordered_map<std::string, std::array<double, 16>>& worldByLink)
{
	return CustomDeviceKinematicModel::forwardLinkWorldById(device, mgr, q, worldByLink);
}

bool applyLinkJointGraph(CustomDeviceBackendData& device, BackendDataManager* mgr, IRobotBackendPoseSink* sink,
						 const std::vector<double>& q)
{
	if (device.links().empty() || device.joints().empty())
	{
		return false;
	}
	if (mgr)
	{
		bool allRestZero = true;
		for (const CustomDeviceJoint& J : device.joints())
		{
			if (std::abs(J.parentToChildRest[3]) > 1e-6 || std::abs(J.parentToChildRest[7]) > 1e-6 ||
				std::abs(J.parentToChildRest[11]) > 1e-6)
			{
				allRestZero = false;
				break;
			}
		}
		if (allRestZero)
		{
			CustomDeviceAssemblyCommit::refreshLinkRestPosesFromGeometry(device, *mgr);
		}
	}
	const BackendMat4 w0Mat = resolveEffectiveDeviceW0(device, mgr);
	double w0[16];
	CustomDeviceMat4Layout::backendMat4ToKinematicCore(w0Mat, w0);
	auto model = CustomDeviceKinematicModel::create(device);
	model->rebuildGraph();
	return model->applyToSink(device, mgr, sink, q, w0);
}
} // namespace

BackendMat4 resolveEffectiveDeviceW0(const CustomDeviceBackendData& device, BackendDataManager* mgr)
{
	if (mgr)
	{
		return device.worldMatrix(mgr);
	}
	return device.baseWorldW0();
}

bool bakeMotionCenterFrameToOriginMm(CustomDeviceAxisConfig& motion, const double parentWorldCm[16],
									 BackendDataManager* mgr)
{
	if (motion.motionCenterFrameBackendId.empty() || !mgr)
	{
		return false;
	}
	const auto frame = mgr->getData(motion.motionCenterFrameBackendId);
	if (!frame || !frame->hasPoseProperty())
	{
		return false;
	}
	const BackendMat4 frameW = frame->worldMatrix(mgr);
	BackendMat4 parentW{};
	std::memcpy(parentW.v, parentWorldCm, sizeof(double) * 16);
	BackendMat4 invParent{};
	if (!backend_mat4_invert_rigid(parentW, invParent))
	{
		return false;
	}
	const BackendVec3 frameOriginWorld = backend_mat4_transform_point(frameW, BackendVec3{0.0, 0.0, 0.0});
	const BackendVec3 local = backend_mat4_transform_point(invParent, frameOriginWorld);
	motion.originMm[0] = local.x;
	motion.originMm[1] = local.y;
	motion.originMm[2] = local.z;
	return true;
}

bool bakeJointMotionOriginFromParentGeometry(CustomDeviceAxisConfig& motion,
											 const std::string& parentGeometryBackendId, BackendDataManager* mgr)
{
	if (parentGeometryBackendId.empty() || !mgr)
	{
		return false;
	}
	const auto parentGeom = mgr->getData(parentGeometryBackendId);
	if (!parentGeom || !parentGeom->hasPoseProperty())
	{
		return false;
	}
	double parentWorldCm[16];
	backendMat4ToArray(parentGeom->worldMatrix(mgr), parentWorldCm);
	return bakeMotionCenterFrameToOriginMm(motion, parentWorldCm, mgr);
}

bool bakeJointMotionOriginFromParentLink(CustomDeviceBackendData& device, CustomDeviceAxisConfig& motion,
										 const std::string& parentLinkId, BackendDataManager* mgr)
{
	if (parentLinkId.empty() || !mgr || !device.usesLinkJointGraph())
	{
		return false;
	}
	device.ensureQSize();
	std::vector<double> q = device.qValues();
	std::unordered_map<std::string, std::array<double, 16>> worldByLink;
	if (!computeLinkWorldMatrices(device, mgr, q, worldByLink))
	{
		return false;
	}
	const auto parentIt = worldByLink.find(parentLinkId);
	if (parentIt == worldByLink.end())
	{
		return false;
	}
	return bakeMotionCenterFrameToOriginMm(motion, parentIt->second.data(), mgr);
}

void rebakeRotateJointOriginsFromFrames(CustomDeviceBackendData& device, BackendDataManager* mgr,
										const std::vector<double>* qForFk)
{
	if (!device.usesLinkJointGraph() || !mgr)
	{
		return;
	}
	device.ensureQSize();
	std::vector<double> q = qForFk ? *qForFk : device.qValues();
	std::vector<CustomDeviceJoint> joints = device.joints();
	if (joints.empty())
	{
		return;
	}

	std::string rootLinkId;
	for (const CustomDeviceLink& L : device.links())
	{
		if (L.fixed)
		{
			rootLinkId = L.id;
			break;
		}
	}
	if (rootLinkId.empty() && !device.links().empty())
	{
		rootLinkId = device.links().front().id;
	}

	std::unordered_map<std::string, std::vector<size_t>> jointsByParent;
	for (size_t ji = 0; ji < joints.size(); ++ji)
	{
		jointsByParent[joints[ji].parentLinkId].push_back(ji);
	}
	std::vector<size_t> jointOrder;
	std::queue<std::string> bfs;
	std::unordered_set<std::string> visitedLinks;
	if (!rootLinkId.empty())
	{
		bfs.push(rootLinkId);
		visitedLinks.insert(rootLinkId);
	}
	while (!bfs.empty())
	{
		const std::string parentId = bfs.front();
		bfs.pop();
		const auto it = jointsByParent.find(parentId);
		if (it == jointsByParent.end())
		{
			continue;
		}
		for (const size_t ji : it->second)
		{
			jointOrder.push_back(ji);
			const std::string& childId = joints[ji].childLinkId;
			if (!visitedLinks.count(childId))
			{
				visitedLinks.insert(childId);
				bfs.push(childId);
			}
		}
	}
	for (size_t ji = 0; ji < joints.size(); ++ji)
	{
		if (std::find(jointOrder.begin(), jointOrder.end(), ji) == jointOrder.end())
		{
			jointOrder.push_back(ji);
		}
	}

	bool changed = false;
	for (const size_t ji : jointOrder)
	{
		CustomDeviceJoint& J = joints[ji];
		if (J.motion.motionType != CustomDeviceMotionType::Rotate || J.motion.motionCenterFrameBackendId.empty())
		{
			continue;
		}
		std::unordered_map<std::string, std::array<double, 16>> worldByLink;
		if (!computeLinkWorldMatrices(device, mgr, q, worldByLink))
		{
			return;
		}
		const auto parentIt = worldByLink.find(J.parentLinkId);
		if (parentIt == worldByLink.end())
		{
			continue;
		}
		const double beforeO[3] = {J.motion.originMm[0], J.motion.originMm[1], J.motion.originMm[2]};
		const bool baked = bakeMotionCenterFrameToOriginMm(J.motion, parentIt->second.data(), mgr);
		if (baked)
		{
			const double delta = std::abs(J.motion.originMm[0] - beforeO[0]) + std::abs(J.motion.originMm[1] - beforeO[1]) +
								 std::abs(J.motion.originMm[2] - beforeO[2]);
			if (delta > 1e-4)
			{
				changed = true;
				device.setJoints(joints);
			}
		}
	}
	if (changed)
	{
		device.setJoints(joints);
	}
}

RobotExternal::RobotExternalAxisConfig toExternalAxisConfig(const CustomDeviceAxisConfig& in)
{
	RobotExternal::RobotExternalAxisConfig out;
	out.enabled = in.enabled;
	out.displayName = in.displayName;
	out.jointName = in.jointName;
	out.motionType = in.motionType == CustomDeviceMotionType::Rotate ? RobotExternal::RobotExternalMotionType::Rotate
																	 : RobotExternal::RobotExternalMotionType::Translate;
	out.kind = out.motionType == RobotExternal::RobotExternalMotionType::Rotate
				   ? RobotExternal::RobotExternalAxisKind::Turntable
				   : RobotExternal::RobotExternalAxisKind::LinearRail;
	out.isPrismatic = out.motionType == RobotExternal::RobotExternalMotionType::Translate;
	out.attachment = RobotExternal::RobotExternalAttachment::RobotBase;
	out.lower = in.lower;
	out.upper = in.upper;
	out.home = in.home;
	out.axis[0] = in.axis[0];
	out.axis[1] = in.axis[1];
	out.axis[2] = in.axis[2];
	out.originMm[0] = in.originMm[0];
	out.originMm[1] = in.originMm[1];
	out.originMm[2] = in.originMm[2];
	return out;
}

RobotExternal::RobotExternalAxisConfigSet toExternalAxisConfigSet(const CustomDeviceAxisConfigSet& in)
{
	RobotExternal::RobotExternalAxisConfigSet out;
	out.axes.reserve(in.axes.size());
	for (const CustomDeviceAxisConfig& a : in.axes)
	{
		out.axes.push_back(toExternalAxisConfig(a));
	}
	return out;
}

bool applyQ(CustomDeviceBackendData& device, BackendDataManager* mgr, IRobotBackendPoseSink* sink,
			const std::vector<double>* qOverride)
{
	if (!device.usesLinkJointGraph() || !mgr)
	{
		return false;
	}
	device.syncAxesFromJoints();
	device.ensureQSize();
	std::vector<double> q = qOverride ? *qOverride : device.qValues();
	if (q.size() < device.axes().axes.size())
	{
		q.resize(device.axes().axes.size(), 0.0);
	}
	for (size_t i = 0; i < device.axes().axes.size() && i < q.size(); ++i)
	{
		q[i] = std::clamp(q[i], device.axes().axes[i].lower, device.axes().axes[i].upper);
	}
	device.setQValues(q);
	rebakeRotateJointOriginsFromFrames(device, mgr, &q);
	return applyLinkJointGraph(device, mgr, sink, q);
}

bool worldPointToDeviceLocalMm(const BackendMat4& w0, const double worldX, const double worldY, const double worldZ,
							   double outLocal[3])
{
	if (!outLocal)
	{
		return false;
	}
	BackendMat4 inv{};
	if (!backend_mat4_invert_rigid(w0, inv))
	{
		return false;
	}
	const BackendVec3 local = backend_mat4_transform_point(inv, BackendVec3{worldX, worldY, worldZ});
	outLocal[0] = local.x;
	outLocal[1] = local.y;
	outLocal[2] = local.z;
	return true;
}

bool worldDirectionToDeviceLocal(const BackendMat4& w0, const double worldDx, const double worldDy, const double worldDz,
								 double outLocal[3])
{
	if (!outLocal)
	{
		return false;
	}
	BackendMat4 inv{};
	if (!backend_mat4_invert_rigid(w0, inv))
	{
		return false;
	}
	const BackendVec3 local = backend_mat4_transform_point(inv, BackendVec3{worldDx, worldDy, worldDz});
	const double n = std::sqrt(local.x * local.x + local.y * local.y + local.z * local.z);
	if (n < 1e-12)
	{
		return false;
	}
	outLocal[0] = local.x / n;
	outLocal[1] = local.y / n;
	outLocal[2] = local.z / n;
	return true;
}

} // namespace CustomDeviceKinematics
