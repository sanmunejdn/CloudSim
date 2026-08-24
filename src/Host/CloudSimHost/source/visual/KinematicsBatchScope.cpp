/// @file KinematicsBatchScope.cpp
/// @brief FK 批量写 worldMatrix 的 RAII 作用域

#include "visual/KinematicsBatchScope.h"

#include "DocumentHost.h"
#include "IDataService.h"
#include "io/CustomDeviceHostOps.h"
#include "io/CustomDeviceRobotMountOps.h"

namespace cloudsim::host
{
KinematicsBatchScope::KinematicsBatchScope(DocumentHost& host) : m_host(host)
{
	m_host.visualSyncEngine().beginKinematicsBatch();
	m_active = true;
}

KinematicsBatchScope::~KinematicsBatchScope()
{
	if (!m_active)
	{
		return;
	}
	m_host.visualSyncEngine().endKinematicsBatch();
	const bool pendingFollow = m_host.visualSyncEngine().takePendingFollowSolveAfterBatch();
	m_host.visualSyncEngine().setPendingFollowSolveAfterBatch(false);
	if (pendingFollow || !m_host.followDirtyBackendIds().empty() || m_host.followSolveForcedPending())
	{
		// FK 批末须全量解跟随：脏集限流会漏掉 TCP 拖动/轴控链式 follower
		m_host.requestFollowSolveForced();
		// 连杆（如 link_6）Data→OSG 后再解 Follow，否则 follower flush 父级陈旧
		(void)m_host.flushVisualSync();
		cloudsim::core::FollowSolveContextDto ctx;
		ctx.skipAll = false;
		(void)m_host.data().runFollowSolveAndSync(ctx, nullptr);
		refreshCustomDevicesFollowingKinematicsTargets(m_host);
	}
}

} // namespace cloudsim::host
