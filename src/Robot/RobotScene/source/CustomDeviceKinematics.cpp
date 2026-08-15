/// @file CustomDeviceKinematics.cpp
/// @brief CustomDeviceKinematics 实现（扁平轴 + Link/Joint 树）

#include "CustomDeviceKinematics.h"

#include "BackendDataManager.h"
#include "BackendFollowMath.h"
#include "FollowAttachmentComponent.h"
#include "IRobotBackendPoseSink.h"

#include "CoreTypes.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <queue>
#include <unordered_map>
#include <unordered_set>

namespace CustomDeviceKinematics
{
namespace
{
void resolveMotionOriginInParentLocal(CustomDeviceAxisConfig& motion, const double parentWorldCm[16],
									  BackendDataManager* mgr)
{
	(void)bakeMotionCenterFrameToOriginMm(motion, parentWorldCm, mgr);
}

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

bool applyLinkJointGraph(CustomDeviceBackendData& device, BackendDataManager* mgr, IRobotBackendPoseSink* sink,
						 const std::vector<double>& q)
{
	const std::vector<CustomDeviceLink>& links = device.links();
	const std::vector<CustomDeviceJoint>& joints = device.joints();
	if (links.empty() || joints.empty())
	{
		return false;
	}

	std::unordered_map<std::string, size_t> linkIndex;
	for (size_t i = 0; i < links.size(); ++i)
	{
		linkIndex[links[i].id] = i;
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

	// childLinkId -> joint index
	std::unordered_map<std::string, size_t> jointByChild;
	for (size_t i = 0; i < joints.size(); ++i)
	{
		jointByChild[joints[i].childLinkId] = i;
	}

	double w0[16];
	backendMat4ToArray(device.baseWorldW0(), w0);
	device.setWorldMatrix(device.baseWorldW0(), mgr);
	if (sink)
	{
		cloudsim::core::Mat4 mat{};
		for (int i = 0; i < 16; ++i)
		{
			mat[static_cast<size_t>(i)] = w0[i];
		}
		sink->setBackendRootWorldMatrixFromWorld(device.id(), mat);
	}

	std::unordered_map<std::string, std::array<double, 16>> worldByLink;
	std::queue<std::string> queue;
	std::unordered_set<std::string> visited;

	{
		std::array<double, 16> wf{};
		RobotExternal::mat4MulColumnMajor16(w0, links[linkIndex[fixedId]].restInDeviceW0, wf.data());
		worldByLink[fixedId] = wf;
		queue.push(fixedId);
		visited.insert(fixedId);
	}

	while (!queue.empty())
	{
		const std::string parentId = queue.front();
		queue.pop();
		for (size_t ji = 0; ji < joints.size(); ++ji)
		{
			const CustomDeviceJoint& J = joints[ji];
			if (J.parentLinkId != parentId)
			{
				continue;
			}
			if (visited.count(J.childLinkId))
			{
				continue;
			}
			if (!linkIndex.count(J.childLinkId) || !worldByLink.count(parentId))
			{
				continue;
			}
			const double qj = ji < q.size() ? q[ji] : J.motion.home;
			CustomDeviceAxisConfig motionCfg = J.motion;
			resolveMotionOriginInParentLocal(motionCfg, worldByLink[parentId].data(), mgr);
			const RobotExternal::RobotExternalAxisConfig ext = toExternalAxisConfig(motionCfg);
			double motion[16];
			RobotExternal::makeAxisMotionColumnMajor(ext, qj, motion);
			double parentMotion[16];
			RobotExternal::mat4MulColumnMajor16(worldByLink[parentId].data(), motion, parentMotion);
			std::array<double, 16> childW{};
			RobotExternal::mat4MulColumnMajor16(parentMotion, J.parentToChildRest, childW.data());
			worldByLink[J.childLinkId] = childW;
			visited.insert(J.childLinkId);
			queue.push(J.childLinkId);
		}
	}

	auto worldQuery = [mgr, sink](const std::string& bid, BackendMat4& out) -> bool {
		if (!mgr)
		{
			return false;
		}
		const auto data = mgr->getData(bid);
		if (!data)
		{
			return false;
		}
		out = data->worldMatrix(mgr);
		(void)sink;
		return true;
	};

	for (const CustomDeviceLink& L : links)
	{
		if (L.geometryBackendId.empty() || !worldByLink.count(L.id))
		{
			continue;
		}
		const auto geom = mgr ? mgr->getData(L.geometryBackendId) : nullptr;
		if (!geom || !geom->hasPoseProperty())
		{
			continue;
		}
		const BackendMat4 wm = arrayToBackendMat4(worldByLink[L.id].data());
		geom->setWorldMatrix(wm, mgr);
		if (sink)
		{
			cloudsim::core::Mat4 mat{};
			for (int i = 0; i < 16; ++i)
			{
				mat[static_cast<size_t>(i)] = worldByLink[L.id][static_cast<size_t>(i)];
			}
			sink->setBackendRootWorldMatrixFromWorld(L.geometryBackendId, mat);
		}
		if (mgr)
		{
			(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(*mgr, worldQuery, *geom, nullptr);
		}
	}
	return true;
}
} // namespace

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
	const double wx = frameW.v[12];
	const double wy = frameW.v[13];
	const double wz = frameW.v[14];
	motion.originMm[0] = invParent.v[0] * wx + invParent.v[4] * wy + invParent.v[8] * wz + invParent.v[12];
	motion.originMm[1] = invParent.v[1] * wx + invParent.v[5] * wy + invParent.v[9] * wz + invParent.v[13];
	motion.originMm[2] = invParent.v[2] * wx + invParent.v[6] * wy + invParent.v[10] * wz + invParent.v[14];
	return true;
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
	return applyLinkJointGraph(device, mgr, sink, q);
}

bool worldPointToDeviceLocalMm(const BackendMat4& w0, const double worldX, const double worldY, const double worldZ,
							   double outLocal[3])
{
	if (!outLocal)
	{
		return false;
	}
	double w0a[16];
	backendMat4ToArray(w0, w0a);
	double inv[16];
	if (!RobotExternal::mat4InvertRigidColumnMajor(w0a, inv))
	{
		return false;
	}
	outLocal[0] = inv[0] * worldX + inv[4] * worldY + inv[8] * worldZ + inv[12];
	outLocal[1] = inv[1] * worldX + inv[5] * worldY + inv[9] * worldZ + inv[13];
	outLocal[2] = inv[2] * worldX + inv[6] * worldY + inv[10] * worldZ + inv[14];
	return true;
}

bool worldDirectionToDeviceLocal(const BackendMat4& w0, const double worldDx, const double worldDy, const double worldDz,
								 double outLocal[3])
{
	if (!outLocal)
	{
		return false;
	}
	double w0a[16];
	backendMat4ToArray(w0, w0a);
	double inv[16];
	if (!RobotExternal::mat4InvertRigidColumnMajor(w0a, inv))
	{
		return false;
	}
	outLocal[0] = inv[0] * worldDx + inv[4] * worldDy + inv[8] * worldDz;
	outLocal[1] = inv[1] * worldDx + inv[5] * worldDy + inv[9] * worldDz;
	outLocal[2] = inv[2] * worldDx + inv[6] * worldDy + inv[10] * worldDz;
	const double n = std::sqrt(outLocal[0] * outLocal[0] + outLocal[1] * outLocal[1] + outLocal[2] * outLocal[2]);
	if (n < 1e-12)
	{
		return false;
	}
	outLocal[0] /= n;
	outLocal[1] /= n;
	outLocal[2] /= n;
	return true;
}

} // namespace CustomDeviceKinematics
