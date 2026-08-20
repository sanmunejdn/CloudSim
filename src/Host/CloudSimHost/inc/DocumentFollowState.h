#ifndef CLOUDSIMHOST_DOCUMENTFOLLOWSTATE_H
#define CLOUDSIMHOST_DOCUMENTFOLLOWSTATE_H

/// @file DocumentFollowState.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief Follow 脏集与帧求解门闩

#include "cloudsim_host_global.h"

#include <string>
#include <unordered_set>

namespace cloudsim::host
{
/// Follow 运行时状态（与场景求解协作，非 Data 组件本身）
class CLOUDSIM_HOST_EXPORT DocumentFollowState
{
public:
	std::unordered_set<std::string>& dirtyBackendIds() { return m_dirtyBackendIds; }
	const std::unordered_set<std::string>& dirtyBackendIds() const { return m_dirtyBackendIds; }
	void clearDirtyBackendIds() { m_dirtyBackendIds.clear(); }

	void requestSolveForced() { m_solveForced = true; }
	bool takeSolveForced()
	{
		const bool v = m_solveForced;
		m_solveForced = false;
		return v;
	}
	bool solveForcedPending() const { return m_solveForced; }

	void setSuppressRobotDirtyNotify(bool suppress) { m_suppressRobotDirtyNotify = suppress; }
	bool suppressRobotDirtyNotify() const { return m_suppressRobotDirtyNotify; }

	void setDeferPropertyPanelVisualFullSync(bool defer) { m_deferPropertyPanelVisualFullSync = defer; }
	bool deferPropertyPanelVisualFullSync() const { return m_deferPropertyPanelVisualFullSync; }

private:
	std::unordered_set<std::string> m_dirtyBackendIds;
	bool m_solveForced = false;
	bool m_suppressRobotDirtyNotify = false;
	bool m_deferPropertyPanelVisualFullSync = false;
};

} // namespace cloudsim::host

#endif
