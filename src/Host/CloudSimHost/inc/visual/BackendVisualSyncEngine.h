#ifndef CLOUDSIMHOST_BACKENDVISUALSYNCENGINE_H
#define CLOUDSIMHOST_BACKENDVISUALSYNCENGINE_H

/// @file BackendVisualSyncEngine.h
/// @brief 后端 worldMatrix → OSG 单轨同步调度

#include "cloudsim_host_global.h"
#include "visual/VisualAspect.h"

#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

class BackendDataManager;
class BackendDataBase;

namespace cloudsim::host
{
class DocumentHost;

class CLOUDSIM_HOST_EXPORT BackendVisualSyncEngine
{
public:
	explicit BackendVisualSyncEngine(DocumentHost& host);

	void markDirty(const std::string& backendId, VisualAspect aspects, VisualChangeReason reason = VisualChangeReason::Manual);
	void clear();

	int kinematicsBatchDepth() const { return m_kinematicsBatchDepth; }
	void beginKinematicsBatch();
	void endKinematicsBatch();

	bool flush(FlushPolicy policy = FlushPolicy::Immediate);
	bool flushTransform(const std::vector<std::string>& orderedIds = {});

	bool deferFollowSolveUntilBatchEnd() const { return m_deferFollowSolveUntilBatchEnd; }
	void setDeferFollowSolveUntilBatchEnd(bool defer) { m_deferFollowSolveUntilBatchEnd = defer; }
	bool takePendingFollowSolveAfterBatch() const;
	void setPendingFollowSolveAfterBatch(bool pending);

private:
	bool flushTransformForId(const std::string& backendId);
	bool flushAppearanceForId(const std::string& backendId);
	bool flushVisibilityForId(const std::string& backendId);
	bool flushGeometryForId(const std::string& backendId);
	std::vector<std::string> resolveTransformFlushOrder(const std::vector<std::string>& hintOrder) const;

	DocumentHost& m_host;
	std::unordered_map<std::string, VisualAspect> m_dirty;
	std::unordered_map<std::string, std::uint64_t> m_lastSyncedGeometryRevision;
	std::unordered_set<std::string> m_batchTransformIds;
	int m_kinematicsBatchDepth = 0;
	bool m_deferFollowSolveUntilBatchEnd = false;
	bool m_pendingFollowSolveAfterBatch = false;
};

} // namespace cloudsim::host

#endif // CLOUDSIMHOST_BACKENDVISUALSYNCENGINE_H
