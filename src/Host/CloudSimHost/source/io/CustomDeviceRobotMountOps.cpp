/// @file CustomDeviceRobotMountOps.cpp
/// @brief 自定义设备机器人法兰挂载（Follow + 位姿）

#include "CustomDeviceRobotMountOps.h"

#include "BackendSceneDocumentFacade.h"
#include "BackendTypeIds.h"
#include "CoreTypes.h"
#include "CustomDeviceBackendData.h"
#include "IDataService.h"
#include "CustomDeviceKinematics.h"
#include "CustomDeviceMat4Layout.h"
#include "CustomDeviceKinematicModel.h"
#include "CustomDeviceRobotMountComponent.h"
#include "io/CustomDeviceHostOps.h"
#include "BackendFollowMath.h"
#include "BackendHierarchyFollow.h"
#include "DocumentHostAccess.h"
#include "OsgWidget.h"
#include "FollowAttachmentComponent.h"
#include "HeadlessRobotContext.h"
#include "IPerLinkRobotStateAccessor.h"
#include "IRobotBackendPoseSink.h"
#include "IRobotUrdfImportContext.h"
#include "RobotCoordinateFrames.h"
#include "RobotMatrixOsgBridge.h"
#include "RobotSceneKinematics.h"
#include "ToolKinematics.h"
#include "UrdfRobotLoader.h"

#include "Adapters.h"

#include <BackendDataManager.h>

#include <array>
#include <algorithm>
#include <cmath>
#include <osg/Matrixd>
#include <queue>
#include <sstream>
#include <unordered_set>

namespace cloudsim::host
{
namespace
{
IRobotBackendPoseSink* poseSinkOf(DocumentHost& host)
{
	if (HeadlessRobotContext* hrc = host.headlessRobotContext())
	{
		return hrc->urdfImportScenePoseSink();
	}
	return host.sceneFacade().poseSink();
}

void runFollowSolveOnHost(DocumentHost& host)
{
	cloudsim::core::FollowSolveContextDto ctx;
	(void)host.data().runFollowSolveAndSync(ctx, nullptr);
}

BackendVec3 backendWorldTranslationMm(const BackendMat4& world)
{
	BackendVec3 pose{};
	BackendVec3 euler{};
	backend_pose_euler_from_world_mat(world, pose, euler);
	return pose;
}

void syncBackendWorldMatrix(DocumentHost& host, BackendDataManager& mgr, const std::string& backendId,
							const BackendMat4& world)
{
	const auto obj = mgr.getData(backendId);
	if (!obj || !obj->hasPoseProperty())
	{
		return;
	}
	obj->setWorldMatrix(world);
	if (IRobotBackendPoseSink* sink = poseSinkOf(host))
	{
		cloudsim::core::Mat4 mat{};
		for (int i = 0; i < 16; ++i)
		{
			mat[static_cast<size_t>(i)] = world.v[static_cast<size_t>(i)];
		}
		sink->setBackendRootWorldMatrixFromWorld(backendId, mat);
	}
}

void syncMountFrameWorldToTcpAlign(DocumentHost& host, BackendDataManager& mgr, const std::string& mountFrameId,
								   const BackendMat4& deviceWorld, const BackendMat4& frameInDevice)
{
	if (mountFrameId.empty())
	{
		return;
	}
	BackendMat4 mountFrameW{};
	if (!backend_mat4_multiply(deviceWorld, frameInDevice, mountFrameW))
	{
		return;
	}
	syncBackendWorldMatrix(host, mgr, mountFrameId, mountFrameW);
}

QString resolveFlangeBackendId(DocumentHost& host, const QString& robotSceneBackendId, QString flangeLinkName)
{
	auto resolveFromLinkMap = [&](const QHash<QString, QString>& linkNameToBackendId) -> QString
	{
		if (!flangeLinkName.isEmpty())
		{
			const QString bid = linkNameToBackendId.value(flangeLinkName);
			if (!bid.isEmpty())
			{
				return bid;
			}
		}
		if (linkNameToBackendId.isEmpty())
		{
			return QString();
		}
		QStringList names = linkNameToBackendId.keys();
		std::sort(names.begin(), names.end());
		return linkNameToBackendId.value(names.last());
	};

	if (HeadlessRobotContext* hrc = host.headlessRobotContext())
	{
		const int idx = hrc->robotInstanceIndexForSceneBackendId(robotSceneBackendId);
		if (idx < 0)
		{
			return QString();
		}
		if (flangeLinkName.isEmpty())
		{
			flangeLinkName = QString::fromStdString(hrc->robotCoordinateFramesForInstance(idx).flangeLinkName);
		}
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (hrc->robotPerLinkKinematicsForInstance(idx, pl))
		{
			const QString bid = resolveFromLinkMap(pl.linkNameToBackendId);
			if (!bid.isEmpty())
			{
				return bid;
			}
		}
		return hrc->robotFlangeBackendId(robotSceneBackendId);
	}

	IRobotUrdfImportContext* ctx = host.robotUrdfImportContext();
	if (!ctx)
	{
		return QString();
	}
	const int idx = ctx->robotInstanceIndexForSceneBackendId(robotSceneBackendId);
	if (idx < 0)
	{
		return QString();
	}
	if (flangeLinkName.isEmpty())
	{
		flangeLinkName = QString::fromStdString(ctx->robotCoordinateFramesForInstance(idx).flangeLinkName);
	}
	if (IPerLinkRobotStateAccessor* accessor = host.perLinkRobotStateAccessor())
	{
		const PerLinkRobotStateSnapshot snap = accessor->extractPerLinkStateSnapshot(idx);
		const QString bid = resolveFromLinkMap(snap.linkNameToBackendId);
		if (!bid.isEmpty())
		{
			return bid;
		}
	}
	return QString();
}

BackendMat4 resolveActiveToolFrameInFlange(DocumentHost& host, const QString& robotSceneBackendId)
{
	IRobotUrdfImportContext* ctx = host.headlessRobotContext();
	if (!ctx)
	{
		ctx = host.robotUrdfImportContext();
	}
	if (!ctx)
	{
		return BackendMat4::identity();
	}
	const int idx = ctx->robotInstanceIndexForSceneBackendId(robotSceneBackendId);
	if (idx < 0)
	{
		return BackendMat4::identity();
	}
	if (const RobotCoordinate::RobotToolFrame* tool =
			RobotCoordinate::activeToolFrame(ctx->robotCoordinateFramesForInstance(idx)))
	{
		return RobotCoordinate::frameToMat4(tool->T_flange_tool);
	}
	return BackendMat4::identity();
}

bool resolveBackendWorldMatrix(DocumentHost& host, BackendDataManager& mgr, const std::string& backendId,
							   BackendMat4& outWorld)
{
	if (IRobotBackendPoseSink* sink = poseSinkOf(host))
	{
		cloudsim::core::Mat4 mat{};
		if (sink->getBackendRootWorldMatrix(backendId, mat))
		{
			for (int i = 0; i < 16; ++i)
			{
				outWorld.v[static_cast<size_t>(i)] = mat[static_cast<size_t>(i)];
			}
			return true;
		}
	}
	const auto obj = mgr.getData(backendId);
	if (!obj)
	{
		return false;
	}
	outWorld = obj->worldMatrix();
	return true;
}

bool tryResolveMountFlangeTcpFromUrdfFk(DocumentHost& host, const QString& robotSceneBackendId,
										  const QString& flangeLinkName, const QString& flangeBackendId,
										  const BackendMat4& toolInFlange, const QVector<double>* jointAnglesOverride,
										  BackendMat4& outFlangeWorld, BackendMat4& outTcpWorld)
{
	IRobotUrdfImportContext* ctx = host.headlessRobotContext();
	if (!ctx)
	{
		ctx = host.robotUrdfImportContext();
	}
	if (!ctx)
	{
		return false;
	}
	const int idx = ctx->robotInstanceIndexForSceneBackendId(robotSceneBackendId);
	if (idx < 0)
	{
		return false;
	}

	QVector<double> jointQ;
	if (jointAnglesOverride && !jointAnglesOverride->isEmpty())
	{
		jointQ = *jointAnglesOverride;
	}
	else if (HeadlessRobotContext* hrc = host.headlessRobotContext())
	{
		QStringList names;
		QVector<double> lower;
		QVector<double> upper;
		if (!hrc->jointMetaForSceneRoot(robotSceneBackendId, names, lower, upper, jointQ))
		{
			return false;
		}
	}
	else
	{
		QVector<double> localQ;
		if (!host.robotLocalJointAnglesForSceneRoot(robotSceneBackendId, localQ))
		{
			return false;
		}
		jointQ = localQ;
	}

	QString urdfPath;
	cloudsim::core::Mat4 basePlacement = cloudsim::core::PlanContextDto::identityMat4();
	QHash<QString, QString> linkNameToBackendId;
	if (HeadlessRobotContext* hrc = host.headlessRobotContext())
	{
		urdfPath = hrc->robotUrdfAbsolutePathForInstance(idx);
		cloudsim::core::RobotPerLinkKinematicsSliceDto pl;
		if (hrc->robotPerLinkKinematicsForInstance(idx, pl))
		{
			basePlacement = pl.robotBasePlacementWorld;
			linkNameToBackendId = pl.linkNameToBackendId;
		}
	}
	else if (IPerLinkRobotStateAccessor* accessor = host.perLinkRobotStateAccessor())
	{
		const PerLinkRobotStateSnapshot snap = accessor->extractPerLinkStateSnapshot(idx);
		urdfPath = snap.urdfAbsolutePath;
		basePlacement = snap.basePlacementWorld;
		linkNameToBackendId = snap.linkNameToBackendId;
	}
	if (urdfPath.isEmpty())
	{
		return false;
	}

	QString resolvedFlangeLink = flangeLinkName.trimmed();
	if (!flangeBackendId.isEmpty())
	{
		for (auto it = linkNameToBackendId.constBegin(); it != linkNameToBackendId.constEnd(); ++it)
		{
			if (it.value() == flangeBackendId)
			{
				resolvedFlangeLink = it.key();
				break;
			}
		}
	}
	if (resolvedFlangeLink.isEmpty())
	{
		resolvedFlangeLink = QString::fromStdString(ctx->robotCoordinateFramesForInstance(idx).flangeLinkName);
	}
	if (resolvedFlangeLink.isEmpty())
	{
		QStringList revoluteChildren;
		(void)UrdfRobotLoader::loadRevoluteJointChildLinksInOrder(urdfPath, revoluteChildren, nullptr);
		if (!revoluteChildren.isEmpty())
		{
			resolvedFlangeLink = revoluteChildren.back();
		}
	}
	if (resolvedFlangeLink.isEmpty())
	{
		return false;
	}

	QHash<QString, osg::Matrixd> linkWorld;
	QString fkErr;
	if (!UrdfRobotLoader::computeLinkWorldMatrices(urdfPath, jointQ, linkWorld, &fkErr) ||
		!linkWorld.contains(resolvedFlangeLink))
	{
		return false;
	}

	const engine::RigidTransform T_base_flange = engine::rigidTransformFromOsg(linkWorld.value(resolvedFlangeLink));
	const engine::RigidTransform T_flange_tool = RobotCoordinate::rigidTransformFromBackendMat4(toolInFlange);
	const engine::RigidTransform T_base_tcp = engine::toolOriginFromFlange(T_base_flange, T_flange_tool);
	const BackendMat4 tcpInBase = RobotCoordinate::backendMat4FromRigidTransform(T_base_tcp);
	const osg::Matrixd P = RobotSceneKinematics::osgMatrixFromCoreMat4(basePlacement);
	const osg::Matrixd flangeOsg = linkWorld.value(resolvedFlangeLink);
	outFlangeWorld = RobotMatrixOsg::backendColMajorFromMatrix(flangeOsg * P);
	outTcpWorld =
		RobotMatrixOsg::backendColMajorFromMatrix(RobotMatrixOsg::matrixFromBackendColMajor(tcpInBase) * P);
	return true;
}

void applyFollowRigidLocal(FollowAttachmentComponent& follow, const BackendMat4& localRel)
{
	BackendVec3 lp{};
	BackendVec3 le{};
	backend_trans_euler_from_rigid_mat(localRel, lp, le);
	follow.setLocalPosition(lp);
	follow.setLocalEulerDeg(le);
}

bool isInDeviceSubtree(BackendDataManager& mgr, const std::string& deviceId, const std::string& backendId)
{
	if (deviceId == backendId)
	{
		return true;
	}
	std::unordered_set<std::string> visited;
	std::queue<std::string> queue;
	queue.push(deviceId);
	visited.insert(deviceId);
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		for (const std::string& child : mgr.childrenOf(cur))
		{
			if (child == backendId)
			{
				return true;
			}
			if (visited.insert(child).second)
			{
				queue.push(child);
			}
		}
	}
	return false;
}

bool resolveDeviceSubtreeParent(BackendDataManager& mgr, const std::string& deviceId, const std::string& childId,
								std::string& outParentId)
{
	for (const std::string& p : mgr.parentsOf(childId))
	{
		if (p == deviceId || isInDeviceSubtree(mgr, deviceId, p))
		{
			outParentId = p;
			return true;
		}
	}
	return false;
}

bool mountFrameWorldHasMeaningfulTranslation(const BackendMat4& m)
{
	const BackendVec3 t = backendWorldTranslationMm(m);
	return std::abs(t.x) > 1e-6 || std::abs(t.y) > 1e-6 || std::abs(t.z) > 1e-6;
}

bool mountFrameMatIsNonIdentity(const BackendMat4& m)
{
	return !backend_mat4_nearly_equal(m, BackendMat4::identity(), 1e-5);
}

const CustomDeviceLink* linkForGeometryBackendId(const CustomDeviceBackendData& device, const std::string& geomId)
{
	for (const CustomDeviceLink& link : device.links())
	{
		if (link.geometryBackendId == geomId)
		{
			return &link;
		}
	}
	return nullptr;
}

bool resolveMountFrameParentWorld(DocumentHost& host, BackendDataManager& mgr, CustomDeviceBackendData& device,
								  const std::string& parentId, BackendMat4& outParentW)
{
	if (parentId.empty())
	{
		return false;
	}
	if (parentId == device.id())
	{
		outParentW = device.worldMatrix();
		return true;
	}
	if (const CustomDeviceLink* link = linkForGeometryBackendId(device, parentId))
	{
		device.ensureQSize();
		std::unordered_map<std::string, std::array<double, 16>> worldByLink;
		if (CustomDeviceKinematicModel::forwardLinkWorldById(device, &mgr, device.qValues(), worldByLink))
		{
			const auto it = worldByLink.find(link->id);
			if (it != worldByLink.end())
			{
				outParentW = CustomDeviceMat4Layout::kinematicCoreToBackendMat4(it->second.data());
				return true;
			}
		}
	}
	const auto parent = mgr.getData(parentId);
	if (!parent || !parent->hasPoseProperty())
	{
		return false;
	}
	outParentW = parent->worldMatrix();
	if (!mountFrameWorldHasMeaningfulTranslation(outParentW))
	{
		BackendMat4 sinkW{};
		if (resolveBackendWorldMatrix(host, mgr, parentId, sinkW) && mountFrameWorldHasMeaningfulTranslation(sinkW))
		{
			outParentW = sinkW;
		}
	}
	return true;
}

bool tryOsgBackendWorld(DocumentHost& host, const std::string& backendId, BackendMat4& outWorld)
{
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		return false;
	}
	osg::Matrixd world;
	if (!osg->getBackendRootWorldMatrix(backendId, world))
	{
		return false;
	}
	for (int c = 0; c < 4; ++c)
	{
		for (int r = 0; r < 4; ++r)
		{
			outWorld.v[static_cast<size_t>(c * 4 + r)] = world(r, c);
		}
	}
	return true;
}

void rebakeMountFrameFollowLocalFromScene(DocumentHost& host, const std::string& mountFrameId)
{
	OsgWidget* osg = osgWidgetFrom(host);
	if (!osg)
	{
		return;
	}
	BackendDataManager& mgr = host.backend();
	const auto frame = mgr.getData(mountFrameId);
	if (!frame || !frame->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
	{
		return;
	}
	const auto worldQuery = [osg](const std::string& bid, BackendMat4& out) -> bool
	{
		osg::Matrixd om;
		if (!osg->getBackendRootWorldMatrix(bid, om))
		{
			return false;
		}
		for (int c = 0; c < 4; ++c)
		{
			for (int r = 0; r < 4; ++r)
			{
				out.v[static_cast<size_t>(c * 4 + r)] = om(r, c);
			}
		}
		return true;
	};
	(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(mgr, worldQuery, *frame, nullptr);
}

bool buildDeviceToMountFrameChain(BackendDataManager& mgr, const std::string& deviceId, const std::string& mountFrameId,
								  std::vector<std::string>& outChain)
{
	outChain.clear();
	std::vector<std::string> rev;
	std::string cur = mountFrameId;
	std::unordered_set<std::string> visited;
	while (cur != deviceId)
	{
		if (!visited.insert(cur).second)
		{
			return false;
		}
		rev.push_back(cur);
		std::string parent;
		if (!resolveDeviceSubtreeParent(mgr, deviceId, cur, parent))
		{
			return false;
		}
		cur = parent;
	}
	rev.push_back(deviceId);
	outChain.assign(rev.rbegin(), rev.rend());
	return !outChain.empty();
}

bool composeLocalChildRelativeToParent(DocumentHost& host, BackendDataManager& mgr, CustomDeviceBackendData& device,
									   const std::string& childId, const std::string& parentId,
									   const BackendMat4& parentW, BackendMat4& outLocal)
{
	const auto child = mgr.getData(childId);
	if (!child)
	{
		return false;
	}
	BackendMat4 childWorld{};
	if (tryOsgBackendWorld(host, childId, childWorld) || resolveBackendWorldMatrix(host, mgr, childId, childWorld))
	{
		BackendMat4 invParent{};
		if (backend_mat4_invert_rigid(parentW, invParent))
		{
			(void)backend_mat4_multiply(invParent, childWorld, outLocal);
			if (mountFrameMatIsNonIdentity(outLocal))
			{
				return true;
			}
		}
	}
	if (const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			child->getComponent(FollowAttachmentComponent::typeKeyStatic())))
	{
		if (follow->enabled() && follow->targetBackendId() == parentId)
		{
			outLocal = backend_world_mat_from_pose(follow->localPosition(), follow->localEulerDeg());
			if (mountFrameMatIsNonIdentity(outLocal))
			{
				return true;
			}
		}
	}
	const BackendPoseValue pv = child->poseValue(BackendPoseReferenceFrame::Parent, &mgr);
	outLocal = backend_world_mat_from_pose(pv.position, pv.eulerDeg);
	return true;
}

bool resolveMountFrameWorldByAncestorChain(DocumentHost& host, BackendDataManager& mgr, CustomDeviceBackendData& device,
										   const std::string& mountFrameId, BackendMat4& outFrameW,
										   std::string& outImmediateParentId, std::string& outAncestorChain)
{
	std::vector<std::string> chain;
	if (!buildDeviceToMountFrameChain(mgr, device.id(), mountFrameId, chain))
	{
		return false;
	}
	{
		std::ostringstream os;
		for (std::size_t i = 0; i < chain.size(); ++i)
		{
			if (i > 0U)
			{
				os << '>';
			}
			os << chain[i];
		}
		outAncestorChain = os.str();
	}
	if (chain.size() >= 2U)
	{
		outImmediateParentId = chain[chain.size() - 2U];
	}

	BackendMat4 world = device.worldMatrix();
	for (std::size_t i = 1; i < chain.size(); ++i)
	{
		const std::string& parentId = chain[i - 1U];
		const std::string& childId = chain[i];
		if (linkForGeometryBackendId(device, childId))
		{
			if (!resolveMountFrameParentWorld(host, mgr, device, childId, world))
			{
				return false;
			}
			continue;
		}
		const BackendMat4 parentW = world;
		BackendMat4 localM = BackendMat4::identity();
		(void)composeLocalChildRelativeToParent(host, mgr, device, childId, parentId, parentW, localM);
		if (!backend_mat4_multiply(parentW, localM, world))
		{
			return false;
		}
	}
	outFrameW = world;
	return true;
}

void syncCustomDeviceSubtreePosesFromOsg(DocumentHost& host, const std::string& deviceRootId)
{
	BackendDataManager& mgr = host.backend();
	IRobotBackendPoseSink* sink = poseSinkOf(host);
	std::queue<std::string> queue;
	queue.push(deviceRootId);
	std::unordered_set<std::string> visited;
	while (!queue.empty())
	{
		const std::string cur = queue.front();
		queue.pop();
		if (!visited.insert(cur).second)
		{
			continue;
		}
		for (const std::string& child : mgr.childrenOf(cur))
		{
			queue.push(child);
			const auto obj = mgr.getData(child);
			if (!obj || !obj->hasPoseProperty())
			{
				continue;
			}
			BackendMat4 osgW{};
			if (!tryOsgBackendWorld(host, child, osgW))
			{
				continue;
			}
			if (backend_mat4_nearly_equal(obj->worldMatrix(), osgW, 1e-5))
			{
				continue;
			}
			obj->setWorldMatrix(osgW);
			if (sink)
			{
				cloudsim::core::Mat4 mat{};
				for (int i = 0; i < 16; ++i)
				{
					mat[static_cast<size_t>(i)] = osgW.v[static_cast<size_t>(i)];
				}
				sink->setBackendRootWorldMatrixFromWorld(child, mat);
			}
		}
	}
}

bool tryResolveMountFrameWorldViaMotionCenterJoint(DocumentHost& host, BackendDataManager& mgr,
												   CustomDeviceBackendData& device, const std::string& mountFrameId,
												   BackendMat4& outFrameW)
{
	for (const CustomDeviceJoint& joint : device.joints())
	{
		if (joint.motion.motionCenterFrameBackendId != mountFrameId)
		{
			continue;
		}
		const CustomDeviceLink* parentLink = nullptr;
		for (const CustomDeviceLink& link : device.links())
		{
			if (link.id == joint.parentLinkId)
			{
				parentLink = &link;
				break;
			}
		}
		if (!parentLink || parentLink->geometryBackendId.empty())
		{
			continue;
		}
		BackendMat4 linkGeomW{};
		if (!resolveMountFrameParentWorld(host, mgr, device, parentLink->geometryBackendId, linkGeomW))
		{
			continue;
		}
		BackendMat4 frameLocal = BackendMat4::identity();
		(void)composeLocalChildRelativeToParent(host, mgr, device, mountFrameId, parentLink->geometryBackendId,
												linkGeomW, frameLocal);
		if (!backend_mat4_multiply(linkGeomW, frameLocal, outFrameW))
		{
			continue;
		}
		return true;
	}
	return false;
}

bool resolveMountFrameWorldForMount(DocumentHost& host, BackendDataManager& mgr, CustomDeviceBackendData& device,
									const std::string& mountFrameId, BackendMat4& outFrameW, std::string& outParentId,
									BackendMat4& outParentW, bool& outUsedHierarchyResolve, std::string& outMgrParentId,
									std::string& outAncestorChain)
{
	outParentId.clear();
	outParentW = BackendMat4::identity();
	outUsedHierarchyResolve = false;
	outMgrParentId.clear();
	outAncestorChain.clear();

	const auto frame = mgr.getData(mountFrameId);
	if (!frame || !frame->hasPoseProperty())
	{
		return false;
	}

	(void)resolveDeviceSubtreeParent(mgr, device.id(), mountFrameId, outMgrParentId);

	BackendMat4 osgFrameW{};
	if (tryOsgBackendWorld(host, mountFrameId, osgFrameW) && mountFrameMatIsNonIdentity(osgFrameW))
	{
		outFrameW = osgFrameW;
		outParentId = outMgrParentId;
		if (!outParentId.empty())
		{
			(void)resolveMountFrameParentWorld(host, mgr, device, outParentId, outParentW);
		}
		outUsedHierarchyResolve = true;
		return true;
	}

	if (resolveMountFrameWorldByAncestorChain(host, mgr, device, mountFrameId, outFrameW, outParentId, outAncestorChain))
	{
		outUsedHierarchyResolve = true;
		if (!outParentId.empty())
		{
			(void)resolveMountFrameParentWorld(host, mgr, device, outParentId, outParentW);
		}
		if (mountFrameMatIsNonIdentity(outFrameW))
		{
			return true;
		}
	}

	if (tryResolveMountFrameWorldViaMotionCenterJoint(host, mgr, device, mountFrameId, outFrameW))
	{
		outUsedHierarchyResolve = true;
		return true;
	}

	const BackendMat4 dataFrameW = frame->worldMatrix();
	if (mountFrameMatIsNonIdentity(dataFrameW))
	{
		outFrameW = dataFrameW;
		return true;
	}

	return resolveBackendWorldMatrix(host, mgr, mountFrameId, outFrameW);
}

void clearConflictingFollowOnMountFrame(DocumentHost& host, BackendDataManager& mgr, const CustomDeviceBackendData& device,
										const std::string& frameBackendId)
{
	const auto frame = mgr.getData(frameBackendId);
	if (!frame || !frame->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
	{
		return;
	}
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		frame->getComponent(FollowAttachmentComponent::typeKeyStatic()));
	if (!follow || !follow->enabled())
	{
		return;
	}
	const std::string targetId = follow->targetBackendId();
	if (!targetId.empty() && !isInDeviceSubtree(mgr, device.id(), targetId))
	{
		frame->removeComponent(FollowAttachmentComponent::typeKeyStatic());
		host.invalidateFollowReverseIndex();
	}
}

void stripHierarchyFollowOnMountFrame(DocumentHost& host, const std::string& frameBackendId)
{
	BackendDataManager& mgr = host.backend();
	const auto frame = mgr.getData(frameBackendId);
	if (!frame || !frame->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
	{
		return;
	}
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		frame->getComponent(FollowAttachmentComponent::typeKeyStatic()));
	if (!follow || !follow->enabled() || !follow->hierarchyDriven())
	{
		return;
	}
	frame->removeComponent(FollowAttachmentComponent::typeKeyStatic());
	host.invalidateFollowReverseIndex();
}

const CustomDeviceLink* linkOwningGeometry(const CustomDeviceBackendData& device, const std::string& backendId)
{
	for (const CustomDeviceLink& link : device.links())
	{
		if (link.geometryBackendId == backendId)
		{
			return &link;
		}
	}
	return nullptr;
}

bool validateMountFrameOnRootOrFixedLink(const CustomDeviceBackendData& device, BackendDataManager& mgr,
										 const std::string& mountFrameId, QString* err)
{
	if (!mgr.contains(mountFrameId))
	{
		if (err)
		{
			*err = QStringLiteral("mount frame backend missing");
		}
		return false;
	}
	if (!isInDeviceSubtree(mgr, device.id(), mountFrameId))
	{
		if (err)
		{
			*err = QStringLiteral("mount frame is not under the device");
		}
		return false;
	}
	const auto mountFrame = mgr.getData(mountFrameId);
	if (!mountFrame || mountFrame->className() != backend_type::kClassFrame)
	{
		if (err)
		{
			*err = QStringLiteral("mount object is not a coordinate frame");
		}
		return false;
	}

	std::string cur = mountFrameId;
	std::unordered_set<std::string> visited;
	while (cur != device.id())
	{
		if (!visited.insert(cur).second)
		{
			if (err)
			{
				*err = QStringLiteral("invalid hierarchy cycle near mount frame");
			}
			return false;
		}
		if (const CustomDeviceLink* link = linkOwningGeometry(device, cur))
		{
			if (!link->fixed)
			{
				if (err)
				{
					*err = QStringLiteral(
						"mount frame must be on the device root or under a fixed link; movable links are not supported in this version");
				}
				return false;
			}
		}
		const std::vector<std::string> parents = mgr.parentsOf(cur);
		std::string parent;
		for (const std::string& p : parents)
		{
			if (p == device.id() || isInDeviceSubtree(mgr, device.id(), p))
			{
				parent = p;
				break;
			}
		}
		if (parent.empty())
		{
			if (err)
			{
				*err = QStringLiteral("mount frame is not under the device");
			}
			return false;
		}
		if (parent == device.id())
		{
			return true;
		}
		if (const CustomDeviceLink* parentLink = linkOwningGeometry(device, parent))
		{
			if (!parentLink->fixed)
			{
				if (err)
				{
					*err = QStringLiteral(
						"mount frame must be on the device root or under a fixed link; movable links are not supported in this version");
				}
				return false;
			}
		}
		cur = parent;
	}
	return true;
}

FollowAttachmentComponent& ensureDeviceRootFollow(CustomDeviceBackendData& device, const std::string& flangeBackendId,
												  const BackendMat4& followLocal)
{
	if (!device.getComponent(FollowAttachmentComponent::typeKeyStatic()))
	{
		device.addComponent(std::make_shared<FollowAttachmentComponent>());
	}
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		device.getComponent(FollowAttachmentComponent::typeKeyStatic()));
	follow->setHierarchyDriven(false);
	follow->setEnabled(true);
	follow->setTargetBackendId(flangeBackendId);
	applyFollowRigidLocal(*follow, followLocal);
	return *follow;
}

void clearDeviceRootFollow(CustomDeviceBackendData& device, DocumentHost& host)
{
	if (!device.hasComponent(FollowAttachmentComponent::typeKeyStatic()))
	{
		return;
	}
	device.removeComponent(FollowAttachmentComponent::typeKeyStatic());
	host.invalidateFollowReverseIndex();
}

bool computeMountFollowLocal(const BackendMat4& toolInFlange, const BackendMat4& frameInDevice,
							 BackendMat4& outFollowLocal)
{
	BackendMat4 invFrameInDevice{};
	if (!backend_mat4_invert_rigid(frameInDevice, invFrameInDevice))
	{
		return false;
	}
	return backend_mat4_multiply(toolInFlange, invFrameInDevice, outFollowLocal);
}

bool computeFollowLocalFromWorldPoses(const BackendMat4& targetWorld, const BackendMat4& followerDesiredWorld,
									  BackendMat4& outFollowLocal)
{
	BackendMat4 invTarget{};
	if (!backend_mat4_invert_rigid(targetWorld, invTarget))
	{
		return false;
	}
	return backend_mat4_multiply(invTarget, followerDesiredWorld, outFollowLocal);
}

BackendMat4 resolveTcpWorldFromFlange(const BackendMat4& flangeWorld, const BackendMat4& toolInFlange)
{
	return RobotCoordinate::targetInBaseFromFlange(flangeWorld, toolInFlange);
}

bool computeDeviceWorldFromTcpAndFrameInDevice(const BackendMat4& tcpWorld, const BackendMat4& frameInDevice,
											   BackendMat4& outDeviceWorld)
{
	BackendMat4 invFrameInDevice{};
	if (!backend_mat4_invert_rigid(frameInDevice, invFrameInDevice))
	{
		return false;
	}
	return backend_mat4_multiply(tcpWorld, invFrameInDevice, outDeviceWorld);
}

bool resolveTcpWorldForMount(DocumentHost& host, const QString& robotSceneBackendId, const QString& flangeLinkName,
							 const QString& flangeBackendId, const BackendMat4& toolInFlange,
							 const QVector<double>* jointAnglesRadForMount, const BackendMat4* mountTcpWorldForAlign,
							 BackendMat4& outTcpWorld)
{
	if (mountTcpWorldForAlign)
	{
		outTcpWorld = *mountTcpWorldForAlign;
		return true;
	}
	BackendMat4 fkFlangeWorld{};
	BackendMat4 fkTcpWorld{};
	if (tryResolveMountFlangeTcpFromUrdfFk(host, robotSceneBackendId, flangeLinkName, flangeBackendId, toolInFlange,
										   jointAnglesRadForMount, fkFlangeWorld, fkTcpWorld))
	{
		outTcpWorld = fkTcpWorld;
		return true;
	}
	BackendDataManager& mgr = host.backend();
	BackendMat4 flangeWorld{};
	if (resolveBackendWorldMatrix(host, mgr, flangeBackendId.toStdString(), flangeWorld))
	{
		outTcpWorld = resolveTcpWorldFromFlange(flangeWorld, toolInFlange);
		return true;
	}
	return false;
}

bool updateMountedDeviceWorldFromRobotTcp(CustomDeviceBackendData& device, DocumentHost& host, BackendDataManager& mgr)
{
	const auto mount = CustomDeviceRobotMountComponent::mountOf(device);
	if (!mount || !mount->enabled())
	{
		return false;
	}
	BackendMat4 tcpWorld{};
	if (!resolveTcpWorldForMount(host, QString::fromStdString(mount->robotSceneBackendId()),
								 QString::fromStdString(mount->flangeLinkName()),
								 QString::fromStdString(mount->flangeBackendId()), mount->toolFrameInFlange(), nullptr,
								 nullptr, tcpWorld))
	{
		return false;
	}
	BackendMat4 deviceWorld{};
	if (!computeDeviceWorldFromTcpAndFrameInDevice(tcpWorld, mount->frameInDeviceW0(), deviceWorld))
	{
		return false;
	}
	device.setWorldMatrix(deviceWorld);
	device.setBaseWorldW0(deviceWorld);
	syncMountFrameWorldToTcpAlign(host, mgr, mount->mountFrameBackendId(), deviceWorld, mount->frameInDeviceW0());

	// 末端位姿变更后须用新 W0 做 FK，并回写各旋转中心 Frame（含链式枢轴）
	IRobotBackendPoseSink* sink = poseSinkOf(host);
	CustomDeviceKinematics::ApplyQOptions applyOpts;
	applyOpts.refreshRestFromGeometry = false;
	applyOpts.rebakeOriginsFromSceneFrames = false;
	(void)CustomDeviceKinematics::applyQ(device, &mgr, sink, nullptr, applyOpts);
	return true;
}

bool rebakeDeviceRootFollowKeepingDeviceWorld(CustomDeviceBackendData& device, DocumentHost& host,
											  CustomDeviceRobotMountComponent& mount, BackendMat4 frameInDevice)
{
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		device.getComponent(FollowAttachmentComponent::typeKeyStatic()));
	if (!follow || !follow->enabled())
	{
		return false;
	}
	BackendDataManager& mgr = host.backend();
	BackendMat4 deviceW{};
	if (!resolveBackendWorldMatrix(host, mgr, device.id(), deviceW))
	{
		deviceW = device.worldMatrix();
	}
	BackendMat4 flangeWorld{};
	if (!resolveBackendWorldMatrix(host, mgr, mount.flangeBackendId(), flangeWorld))
	{
		return false;
	}
	BackendMat4 followLocal{};
	if (!computeFollowLocalFromWorldPoses(flangeWorld, deviceW, followLocal))
	{
		return false;
	}
	mount.setFrameInDeviceW0(frameInDevice);
	mount.setAlignMountFrameToTcp(false);
	applyFollowRigidLocal(*follow, followLocal);
	host.markFollowAttachmentDirtyFromBackendMove(mount.flangeBackendId());
	return true;
}

QString resolveMountFrameBackendId(DocumentHost& host, const CustomDeviceBackendData& device, QString explicitId)
{
	explicitId = explicitId.trimmed();
	if (!explicitId.isEmpty())
	{
		return explicitId;
	}
	BackendDataManager& mgr = host.backend();
	const std::string deviceId = device.id();
	std::vector<std::string> subtreeFrames;
	{
		std::unordered_set<std::string> visited;
		std::queue<std::string> queue;
		queue.push(deviceId);
		visited.insert(deviceId);
		while (!queue.empty())
		{
			const std::string cur = queue.front();
			queue.pop();
			for (const std::string& child : mgr.childrenOf(cur))
			{
				const auto data = mgr.getData(child);
				if (data && data->className() == backend_type::kClassFrame)
				{
					subtreeFrames.push_back(child);
				}
				if (visited.insert(child).second)
				{
					queue.push(child);
				}
			}
		}
	}
	if (!subtreeFrames.empty())
	{
		return QString::fromStdString(subtreeFrames.front());
	}
	for (const CustomDeviceJoint& joint : device.joints())
	{
		if (!joint.motion.motionCenterFrameBackendId.empty() &&
			mgr.contains(joint.motion.motionCenterFrameBackendId))
		{
			return QString::fromStdString(joint.motion.motionCenterFrameBackendId);
		}
	}
	for (const auto& obj : host.listObjects())
	{
		if (obj && obj->className() == backend_type::kClassFrame)
		{
			return QString::fromStdString(obj->id());
		}
	}
	return QString();
}

bool rebakeDeviceRootFollowLocal(CustomDeviceBackendData& device, DocumentHost& host)
{
	const auto mount = CustomDeviceRobotMountComponent::mountOf(device);
	if (!mount || !mount->enabled() || mount->flangeBackendId().empty())
	{
		return false;
	}
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		device.getComponent(FollowAttachmentComponent::typeKeyStatic()));
	if (!follow || !follow->enabled())
	{
		return false;
	}
	BackendMat4 toolInFlange = resolveActiveToolFrameInFlange(host, QString::fromStdString(mount->robotSceneBackendId()));
	mount->setToolFrameInFlange(toolInFlange);
	BackendMat4 followLocal{};
	if (!computeMountFollowLocal(toolInFlange, mount->frameInDeviceW0(), followLocal))
	{
		return false;
	}
	applyFollowRigidLocal(*follow, followLocal);
	host.markFollowAttachmentDirtyFromBackendMove(mount->flangeBackendId());
	return true;
}
} // namespace

bool mountCustomDeviceToFlange(CustomDeviceBackendData& device, DocumentHost& host,
							   const QString& robotSceneBackendId, const QString& flangeLinkName,
							   const QString& flangeBackendIdIn, const QString& mountFrameBackendIdIn,
							   const BackendMat4& toolFrameInFlange,
							   const QVector<double>* localJointAnglesRadForMount,
							   const BackendMat4* mountTcpWorldForAlign, QString* err)
{
	if (!device.usesLinkJointGraph())
	{
		if (err)
		{
			*err = QStringLiteral("device has no Link/Joint graph");
		}
		return false;
	}
	const QString mountFrameBackendId = resolveMountFrameBackendId(host, device, mountFrameBackendIdIn);
	if (mountFrameBackendId.isEmpty())
	{
		if (err)
		{
			*err = QStringLiteral("no mount frame found; create a coordinate frame under the device");
		}
		return false;
	}

	BackendDataManager& mgr = host.backend();
	if (!validateMountFrameOnRootOrFixedLink(device, mgr, mountFrameBackendId.toStdString(), err))
	{
		return false;
	}
	clearConflictingFollowOnMountFrame(host, mgr, device, mountFrameBackendId.toStdString());

	QString flangeBackendId = flangeBackendIdIn;
	if (flangeBackendId.isEmpty())
	{
		flangeBackendId = resolveFlangeBackendId(host, robotSceneBackendId, flangeLinkName);
	}
	if (flangeBackendId.isEmpty())
	{
		if (err)
		{
			*err = QStringLiteral("flange backend not found");
		}
		return false;
	}
	if (!mgr.getData(flangeBackendId.toStdString()))
	{
		if (err)
		{
			*err = QStringLiteral("flange backend missing");
		}
		return false;
	}

	IRobotBackendPoseSink* sink = poseSinkOf(host);
	(void)host.flushVisualSync();
	CustomDeviceKinematics::ApplyQOptions preMountApplyQ;
	preMountApplyQ.refreshRestFromGeometry = false;
	(void)CustomDeviceKinematics::applyQ(device, &mgr, sink, nullptr, preMountApplyQ);
	syncCustomDeviceSubtreePosesFromOsg(host, device.id());
	rebakeMountFrameFollowLocalFromScene(host, mountFrameBackendId.toStdString());
	host.markFollowAttachmentDirtyFromBackendMove(device.id());
	runFollowSolveOnHost(host);

	BackendMat4 deviceW{};
	BackendMat4 frameW{};
	std::string mountFrameParentId;
	std::string mountFrameMgrParentId;
	std::string mountFrameAncestorChain;
	BackendMat4 mountFrameParentW = BackendMat4::identity();
	bool usedHierarchyResolve = false;
	if (!resolveMountFrameWorldForMount(host, mgr, device, mountFrameBackendId.toStdString(), frameW, mountFrameParentId,
									  mountFrameParentW, usedHierarchyResolve, mountFrameMgrParentId,
									  mountFrameAncestorChain))
	{
		if (err)
		{
			*err = QStringLiteral("mount frame backend missing");
		}
		return false;
	}
	if (!resolveBackendWorldMatrix(host, mgr, device.id(), deviceW))
	{
		deviceW = device.worldMatrix();
	}
	stripHierarchyFollowOnMountFrame(host, mountFrameBackendId.toStdString());

	BackendMat4 toolInFlange = toolFrameInFlange;
	if (backend_mat4_nearly_equal(toolInFlange, BackendMat4::identity(), 1e-6))
	{
		toolInFlange = resolveActiveToolFrameInFlange(host, robotSceneBackendId);
	}

	BackendMat4 frameInDevice{};
	if (!CustomDeviceRobotMountComponent::computeFrameInDeviceFromWorldPoses(deviceW, frameW, frameInDevice))
	{
		if (err)
		{
			*err = QStringLiteral("failed to compute frameInDeviceW0");
		}
		return false;
	}

	BackendMat4 tcpWorld{};
	if (!resolveTcpWorldForMount(host, robotSceneBackendId, flangeLinkName, flangeBackendId, toolInFlange,
								 localJointAnglesRadForMount, mountTcpWorldForAlign, tcpWorld))
	{
		if (err)
		{
			*err = QStringLiteral("failed to resolve TCP world pose");
		}
		return false;
	}

	BackendMat4 deviceWorldDesired{};
	if (!computeDeviceWorldFromTcpAndFrameInDevice(tcpWorld, frameInDevice, deviceWorldDesired))
	{
		if (err)
		{
			*err = QStringLiteral("failed to compute device world from TCP");
		}
		return false;
	}

	BackendMat4 followLocal{};
	if (!computeMountFollowLocal(toolInFlange, frameInDevice, followLocal))
	{
		if (err)
		{
			*err = QStringLiteral("failed to compute device follow local offset");
		}
		return false;
	}

	device.setWorldMatrix(deviceWorldDesired);
	device.setBaseWorldW0(deviceWorldDesired);
	ensureDeviceRootFollow(device, flangeBackendId.toStdString(), followLocal);
	host.invalidateFollowReverseIndex();

	CustomDeviceRobotMountComponent& mount = CustomDeviceRobotMountComponent::ensureMount(device);
	mount.setEnabled(true);
	mount.setRobotSceneBackendId(robotSceneBackendId.toStdString());
	mount.setFlangeLinkName(flangeLinkName.toStdString());
	mount.setFlangeBackendId(flangeBackendId.toStdString());
	mount.setMountFrameBackendId(mountFrameBackendId.toStdString());
	mount.setFrameInDeviceW0(frameInDevice);
	mount.setToolFrameInFlange(toolInFlange);
	mount.setAlignMountFrameToTcp(false);

	syncMountFrameWorldToTcpAlign(host, mgr, mountFrameBackendId.toStdString(), deviceWorldDesired, frameInDevice);

	host.markFollowAttachmentDirtyFromBackendMove(flangeBackendId.toStdString());
	host.markFollowAttachmentDirtyFromBackendMove(device.id());
	syncCustomDeviceKinematicsAfterRootPoseChange(host, device.id());
	(void)host.syncOuterPatFromBackendId(mountFrameBackendId.toStdString());
	return true;
}

bool unmountCustomDeviceFromRobot(CustomDeviceBackendData& device, DocumentHost& host, QString* err)
{
	CustomDeviceRobotMountComponent* mount = CustomDeviceRobotMountComponent::mountOf(device).get();
	if (!mount || !mount->enabled())
	{
		if (err)
		{
			*err = QStringLiteral("device is not mounted");
		}
		return false;
	}
	mount->setEnabled(false);
	mount->setAlignMountFrameToTcp(false);
	clearDeviceRootFollow(device, host);
	syncCustomDeviceKinematicsAfterRootPoseChange(host, device.id());
	return true;
}

void refreshCustomDevicesFollowingKinematicsTargets(DocumentHost& host)
{
	BackendDataManager& mgr = host.backend();
	for (const auto& obj : host.listObjects())
	{
		if (!obj || obj->className() != backend_type::kClassCustomDevice)
		{
			continue;
		}
		const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(obj);
		if (!device)
		{
			continue;
		}
		const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			device->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!follow || !follow->enabled() || follow->targetBackendId().empty())
		{
			continue;
		}
		if (!host.isKinematicsOwnedBackend(follow->targetBackendId()))
		{
			continue;
		}
		const auto mount = CustomDeviceRobotMountComponent::mountOf(*device);
		if (!updateMountedDeviceWorldFromRobotTcp(*device, host, mgr))
		{
			continue;
		}
		(void)host.syncOuterPatFromBackendId(device->id());
		flushCustomDeviceLinkGeometryVisual(host, device->id());
		flushCustomDeviceMotionCenterFrameVisual(host, device->id());
		if (mount && !mount->mountFrameBackendId().empty())
		{
			(void)host.syncOuterPatFromBackendId(mount->mountFrameBackendId());
		}
	}
}

bool rebakeMountedDeviceFromInstallFramePose(DocumentHost& host, const std::string& frameBackendId)
{
	if (frameBackendId.empty())
	{
		return false;
	}
	BackendDataManager& mgr = host.backend();
	for (const auto& obj : host.listObjects())
	{
		if (!obj || obj->className() != backend_type::kClassCustomDevice)
		{
			continue;
		}
		const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(obj);
		if (!device)
		{
			continue;
		}
		const auto mount = CustomDeviceRobotMountComponent::mountOf(*device);
		if (!mount || !mount->enabled() || mount->mountFrameBackendId() != frameBackendId)
		{
			continue;
		}
		BackendMat4 deviceW{};
		BackendMat4 frameW{};
		if (!resolveBackendWorldMatrix(host, mgr, device->id(), deviceW))
		{
			deviceW = device->worldMatrix();
		}
		if (!resolveBackendWorldMatrix(host, mgr, frameBackendId, frameW))
		{
			const auto frame = mgr.getData(frameBackendId);
			if (!frame)
			{
				return false;
			}
			frameW = frame->worldMatrix();
		}
		BackendMat4 frameInDevice{};
		if (!CustomDeviceRobotMountComponent::computeFrameInDeviceFromWorldPoses(deviceW, frameW, frameInDevice))
		{
			return false;
		}
		return rebakeDeviceRootFollowKeepingDeviceWorld(*device, host, *mount, frameInDevice);
	}
	return false;
}

void rebakeMountedCustomDevicesFollowLocals(DocumentHost& host)
{
	bool anyRebaked = false;
	for (const auto& obj : host.listObjects())
	{
		if (!obj || obj->className() != backend_type::kClassCustomDevice)
		{
			continue;
		}
		const auto device = std::dynamic_pointer_cast<CustomDeviceBackendData>(obj);
		if (!device)
		{
			continue;
		}
		if (rebakeDeviceRootFollowLocal(*device, host))
		{
			anyRebaked = true;
		}
	}
	if (!anyRebaked)
	{
		return;
	}
	runFollowSolveOnHost(host);
	refreshCustomDevicesFollowingKinematicsTargets(host);
}

void refreshMountedCustomDevicesAfterRobotFk(DocumentHost& host)
{
	refreshCustomDevicesFollowingKinematicsTargets(host);
}

} // namespace cloudsim::host
