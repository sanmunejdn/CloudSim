/// @file BackendFollowSolve.cpp
/// @brief Follow 脏集求解 + compound 传播

#include "BackendFollowSolve.h"

#include "BackendCompoundPropagate.h"
#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "BackendFollowTransformSolver.h"
#include "BackendSceneDocumentFacade.h"
#include "BackendTypeIds.h"
#include "CoreTypes.h"
#include "CustomDeviceBackendData.h"
#include "CustomDeviceKinematics.h"
#include "CustomDeviceRobotMountComponent.h"
#include "DocumentHost.h"
#include "FollowAttachmentComponent.h"
#include "HeadlessRobotContext.h"
#include "IRenderView.h"
#include "IRobotBackendPoseSink.h"
#include "OsgWidget.h"
#include "OsgWidgetSceneBridge.h"
#include "io/CustomDeviceRobotMountOps.h"
#include "visual/VisualAspect.h"

#include "PropertyBag.h"

#include <QString>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <osg/Matrixd>

namespace cloudsim::host
{
namespace
{
std::string trimUtf8Whitespace(const std::string& s)
{
	std::size_t a = 0;
	std::size_t b = s.size();
	while (a < b && (s[a] == ' ' || s[a] == '\t' || s[a] == '\r' || s[a] == '\n'))
	{
		++a;
	}
	while (b > a && (s[b - 1] == ' ' || s[b - 1] == '\t' || s[b - 1] == '\r' || s[b - 1] == '\n'))
	{
		--b;
	}
	return s.substr(a, b - a);
}

void resolveFollowTargetsFromNames(BackendDataManager& mgr)
{
	for (const auto& d : mgr.listData())
	{
		if (!d)
		{
			continue;
		}
		auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!comp || !comp->enabled() || !comp->targetBackendId().empty())
		{
			continue;
		}
		std::string name;
		if (!d->propertyBag().tryGet<std::string>("follow.targetName", name))
		{
			continue;
		}
		name = trimUtf8Whitespace(name);
		if (name.empty())
		{
			continue;
		}
		const std::vector<std::shared_ptr<BackendDataBase>> matches = mgr.findByName(name);
		if (matches.size() != 1U || !matches.front())
		{
			continue;
		}
		comp->setTargetBackendId(matches.front()->id());
	}
}

bool dirtyContainsKinematicsSeed(const DocumentHost& page, const std::unordered_set<std::string>& dirty)
{
	for (const std::string& id : dirty)
	{
		if (page.isKinematicsOwnedBackend(id))
		{
			return true;
		}
	}
	return false;
}

IRobotBackendPoseSink* poseSinkOf(DocumentHost& host)
{
	if (HeadlessRobotContext* hrc = host.headlessRobotContext())
	{
		return hrc->urdfImportScenePoseSink();
	}
	return host.sceneFacade().poseSink();
}

backend_compound::WorldWriteFn makePoseSinkWriter(IRobotBackendPoseSink* sink)
{
	if (!sink)
	{
		return nullptr;
	}
	return [sink](const std::string& id, const BackendMat4& wm)
	{
		cloudsim::core::Mat4 mat{};
		for (int k = 0; k < 16; ++k)
		{
			mat[static_cast<size_t>(k)] = wm.v[k];
		}
		sink->setBackendRootWorldMatrixFromWorld(id, mat);
	};
}

bool isMountedCustomDevice(const BackendDataBase& data)
{
	if (data.className() != backend_type::kClassCustomDevice)
	{
		return false;
	}
	const auto device = dynamic_cast<const CustomDeviceBackendData*>(&data);
	const auto mount = device ? CustomDeviceRobotMountComponent::mountOf(*device) : nullptr;
	return mount && mount->enabled();
}

bool isEnabledFollower(const BackendDataBase& data)
{
	const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		data.getComponent(FollowAttachmentComponent::typeKeyStatic()));
	return follow && follow->enabled() && !follow->targetBackendId().empty();
}

std::unordered_set<std::string> collectEnabledFollowerIds(BackendDataManager& mgr)
{
	std::unordered_set<std::string> out;
	for (const auto& d : mgr.listData())
	{
		if (d && isEnabledFollower(*d))
		{
			out.insert(d->id());
		}
	}
	return out;
}

std::unordered_map<std::string, BackendMat4> snapshotWorlds(BackendDataManager& mgr,
															const std::unordered_set<std::string>& ids)
{
	std::unordered_map<std::string, BackendMat4> out;
	for (const std::string& id : ids)
	{
		const auto obj = mgr.getData(id);
		if (!obj || !obj->hasPoseProperty())
		{
			continue;
		}
		out[id] = obj->worldMatrix();
	}
	return out;
}

std::unordered_set<std::string> followersTargeting(BackendDataManager& mgr,
												   const std::unordered_set<std::string>& targets)
{
	std::unordered_set<std::string> out;
	if (targets.empty())
	{
		return out;
	}
	for (const auto& d : mgr.listData())
	{
		if (!d)
		{
			continue;
		}
		const auto follow = std::dynamic_pointer_cast<FollowAttachmentComponent>(
			d->getComponent(FollowAttachmentComponent::typeKeyStatic()));
		if (!follow || !follow->enabled())
		{
			continue;
		}
		const std::string tid = follow->targetBackendId();
		if (!tid.empty() && targets.count(tid) != 0)
		{
			out.insert(d->id());
		}
	}
	return out;
}

/// Follow 写根后：未挂载设备走 applyQ；其它对象走 compound
std::unordered_set<std::string> propagateAfterFollowMovedRoot(DocumentHost& page, BackendDataManager& mgr,
															  const std::string& rootId, const BackendMat4& wOld)
{
	std::unordered_set<std::string> touched;
	const auto obj = mgr.getData(rootId);
	if (!obj || !obj->hasPoseProperty())
	{
		return touched;
	}
	if (isMountedCustomDevice(*obj))
	{
		return touched;
	}
	const BackendMat4 wNew = obj->worldMatrix();
	if (backend_mat4_nearly_equal(wOld, wNew, 1e-9))
	{
		return touched;
	}

	IRobotBackendPoseSink* sink = poseSinkOf(page);
	if (obj->className() == backend_type::kClassCustomDevice)
	{
		auto* device = dynamic_cast<CustomDeviceBackendData*>(obj.get());
		if (!device || !device->usesLinkJointGraph())
		{
			return touched;
		}
		device->setBaseWorldW0(wNew);
		CustomDeviceKinematics::ApplyQOptions opts;
		opts.refreshRestFromGeometry = false;
		opts.rebakeOriginsFromSceneFrames = false;
		(void)CustomDeviceKinematics::applyQ(*device, &mgr, sink, nullptr, opts);
		touched.insert(rootId);
		for (const CustomDeviceLink& L : device->links())
		{
			if (!L.geometryBackendId.empty())
			{
				touched.insert(L.geometryBackendId);
			}
		}
		// P3-4: 递归收集所有子孙，不再硬编码两级
		std::vector<std::string> stack;
		for (const std::string& child : mgr.childrenOf(rootId))
		{
			stack.push_back(child);
		}
		while (!stack.empty())
		{
			const std::string u = stack.back();
			stack.pop_back();
			if (!touched.insert(u).second)
			{
				continue;
			}
			for (const std::string& c : mgr.childrenOf(u))
			{
				stack.push_back(c);
			}
		}
		return touched;
	}

	return backend_compound::propagateFromWorldChange(mgr, rootId, wOld, wNew, nullptr, makePoseSinkWriter(sink));
}

void markVisualForFollowers(DocumentHost& page, BackendDataManager& mgr, const std::string& gizmoDragSelectedId,
							const std::string& manualAuthorityId, const bool usePoseLimit,
							const std::unordered_set<std::string>& dirty)
{
	for (const auto& d : mgr.listData())
	{
		if (!d)
		{
			continue;
		}
		if (page.isKinematicsOwnedBackend(d->id()))
		{
			continue;
		}
		if (!isEnabledFollower(*d))
		{
			continue;
		}
		const std::string fid = d->id();
		if (!gizmoDragSelectedId.empty() && fid == gizmoDragSelectedId)
		{
			continue;
		}
		if (!manualAuthorityId.empty() && fid == manualAuthorityId)
		{
			continue;
		}
		if (usePoseLimit && !dirty.count(fid))
		{
			continue;
		}
		page.markVisualDirty(fid, VisualAspect::Transform);
	}
	if (!usePoseLimit)
	{
		return;
	}
	for (const std::string& id : dirty)
	{
		if (page.isKinematicsOwnedBackend(id))
		{
			continue;
		}
		if (!gizmoDragSelectedId.empty() && id == gizmoDragSelectedId)
		{
			continue;
		}
		if (!manualAuthorityId.empty() && id == manualAuthorityId)
		{
			continue;
		}
		const auto obj = mgr.getData(id);
		if (!obj || !obj->hasPoseProperty())
		{
			continue;
		}
		page.markVisualDirty(id, VisualAspect::Transform);
	}
}
} // namespace

std::unordered_set<std::string> propagateCompoundAfterRootWorldChange(DocumentHost& host, const std::string& rootId,
																	  const BackendMat4& wOld, const BackendMat4& wNew)
{
	BackendDataManager& mgr = host.backend();
	const auto root = mgr.getData(rootId);
	if (!root || rootId.empty())
	{
		return {};
	}
	if (root->className() == backend_type::kClassCustomDevice)
	{
		return {};
	}
	IRobotBackendPoseSink* sink = poseSinkOf(host);
	auto touched =
		backend_compound::propagateFromWorldChange(mgr, rootId, wOld, wNew, nullptr, makePoseSinkWriter(sink));
	for (const std::string& id : touched)
	{
		host.markVisualDirty(id, VisualAspect::Transform);
		host.markFollowAttachmentDirtyFromBackendMove(id);
	}
	return touched;
}

void runBackendFollowSolveAndSync(DocumentHost& page, OsgWidget* osg, const FollowSolveContext* ctx,
								  const std::string* manualPoseAuthorityBackendId)
{
	if (ctx && ctx->skipAll && ctx->skipAll())
	{
		return;
	}

	page.stripKinematicsOwnedFollowAttachments();
	page.stripHierarchyDrivenFollowAttachments();

	BackendDataManager& mgr = page.backend();
	resolveFollowTargetsFromNames(mgr);
	bool forced = page.takeFollowSolveForced();
	auto& dirty = page.followDirtyBackendIds();
	if (!forced && dirtyContainsKinematicsSeed(page, dirty))
	{
		forced = true;
	}
	const bool gizmoDrag = osg && osg->isTransformGizmoDragging() && !osg->isTcpDragTeachActive();
	if (!forced && dirty.empty())
	{
		return;
	}

	std::string skipId;
	std::string gizmoDragSelectedId;
	std::string manualAuthorityId;
	if (manualPoseAuthorityBackendId && !manualPoseAuthorityBackendId->empty())
	{
		manualAuthorityId = *manualPoseAuthorityBackendId;
		skipId = manualAuthorityId;
	}
	if (gizmoDrag && ctx && ctx->fillGizmoSelectedId && ctx->fillGizmoSelectedId(gizmoDragSelectedId))
	{
		skipId = gizmoDragSelectedId;
	}
	if (!skipId.empty())
	{
		bakeFollowLocalAfterManualPoseEdit(page, skipId);
	}

	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [&mgr](const std::string& bid,
																		 BackendMat4& out) -> bool
	{
		const auto obj = mgr.getData(bid);
		if (!obj || !obj->hasPoseProperty())
		{
			return false;
		}
		out = obj->worldMatrix();
		return true;
	};

	const bool usePoseLimit = !forced && !dirty.empty();
	const std::unordered_set<std::string>* limitPtr = usePoseLimit ? &dirty : nullptr;

	const std::unordered_set<std::string> followerIds = collectEnabledFollowerIds(mgr);
	std::unordered_set<std::string> snapshotIds = followerIds;
	if (usePoseLimit)
	{
		snapshotIds.clear();
		for (const std::string& id : dirty)
		{
			if (followerIds.count(id) != 0)
			{
				snapshotIds.insert(id);
			}
		}
	}
	const auto worldsBefore = snapshotWorlds(mgr, snapshotIds);

	BackendFollowTransformSolver::solve(mgr, worldQuery, skipId, limitPtr);

	std::unordered_set<std::string> compoundTouched;
	for (const auto& kv : worldsBefore)
	{
		const std::string& fid = kv.first;
		if (!skipId.empty() && fid == skipId)
		{
			continue;
		}
		auto part = propagateAfterFollowMovedRoot(page, mgr, fid, kv.second);
		compoundTouched.insert(part.begin(), part.end());
	}

	const std::unordered_set<std::string> secondPassFollowers = followersTargeting(mgr, compoundTouched);
	if (!secondPassFollowers.empty())
	{
		BackendFollowTransformSolver::solve(mgr, worldQuery, skipId, &secondPassFollowers);
		for (const std::string& fid : secondPassFollowers)
		{
			page.markVisualDirty(fid, VisualAspect::Transform);
		}
	}

	markVisualForFollowers(page, mgr, gizmoDragSelectedId, manualAuthorityId, usePoseLimit, dirty);
	for (const std::string& id : compoundTouched)
	{
		page.markVisualDirty(id, VisualAspect::Transform);
	}

	refreshCustomDevicesFollowingKinematicsTargets(page);

	// 挂载 applyQ/compound 之后，再解「跟连杆子 Solid」的跨部件 Follow
	std::unordered_set<std::string> mountTouched;
	for (const auto& obj : page.listObjects())
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
		if (!mount || !mount->enabled())
		{
			continue;
		}
		mountTouched.insert(device->id());
		for (const CustomDeviceLink& L : device->links())
		{
			if (L.geometryBackendId.empty())
			{
				continue;
			}
			mountTouched.insert(L.geometryBackendId);
			for (const std::string& child : mgr.childrenOf(L.geometryBackendId))
			{
				mountTouched.insert(child);
			}
		}
	}
	const std::unordered_set<std::string> postMountFollowers = followersTargeting(mgr, mountTouched);
	if (!postMountFollowers.empty())
	{
		BackendFollowTransformSolver::solve(mgr, worldQuery, skipId, &postMountFollowers);
		for (const std::string& fid : postMountFollowers)
		{
			page.markVisualDirty(fid, VisualAspect::Transform);
		}
	}

	page.flushVisualSync();
	dirty.clear();
}

void afterFollowPropertyEdited(DocumentHost& host, const QString& backendId, const QString& propertyKey,
							   const QString& valueText)
{
	const std::string id = backendId.toStdString();
	const auto data = host.backend().getData(id);
	if (!data)
	{
		return;
	}
	if (host.isKinematicsOwnedBackend(id))
	{
		if (data->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
		{
			data->removeComponent(FollowAttachmentComponent::typeKeyStatic());
			host.invalidateFollowReverseIndex();
		}
		return;
	}
	if (!data->hasComponent(FollowAttachmentComponent::typeKeyStatic()))
	{
		host.invalidateFollowReverseIndex();
		return;
	}
	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [&host](const std::string& bid,
																		   BackendMat4& out) -> bool
	{
		const auto obj = host.backend().getData(bid);
		if (!obj || !obj->hasPoseProperty())
		{
			return false;
		}
		out = obj->worldMatrix();
		return true;
	};
	if (propertyKey == QStringLiteral("follow.targetId") || propertyKey == QStringLiteral("follow.targetName") ||
		(propertyKey == QStringLiteral("follow.enabled") &&
		 (valueText == QStringLiteral("1") || valueText.compare(QStringLiteral("true"), Qt::CaseInsensitive) == 0)))
	{
		(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(host.backend(), worldQuery, *data, nullptr);
	}
	host.markFollowAttachmentDirtyFromBackendMove(id);
	host.invalidateFollowReverseIndex();
}

void bakeFollowLocalAfterManualPoseEdit(DocumentHost& host, const std::string& backendId)
{
	if (backendId.empty() || host.isKinematicsOwnedBackend(backendId))
	{
		return;
	}
	const auto data = host.backend().getData(backendId);
	if (!data)
	{
		return;
	}
	const auto comp = std::dynamic_pointer_cast<FollowAttachmentComponent>(
		data->getComponent(FollowAttachmentComponent::typeKeyStatic()));
	if (!comp || !comp->enabled() || comp->targetBackendId().empty())
	{
		return;
	}
	const BackendFollowTransformSolver::WorldMatQuery worldQuery = [&host](const std::string& bid,
																		   BackendMat4& out) -> bool
	{
		const auto obj = host.backend().getData(bid);
		if (!obj || !obj->hasPoseProperty())
		{
			return false;
		}
		out = obj->worldMatrix();
		return true;
	};
	(void)FollowAttachmentComponent::recomputeLocalFromCurrentWorld(host.backend(), worldQuery, *data, nullptr);
}

} // namespace cloudsim::host
