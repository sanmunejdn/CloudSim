/// @file BackendVisualSyncEngine.cpp
/// @brief worldMatrix → OSG 单轨 flush 调度

#include "visual/BackendVisualSyncEngine.h"

#include "BackendDataBase.h"
#include "BackendDataManager.h"
#include "DocumentHost.h"
#include "DocumentHostAccess.h"
#include "OsgWidget.h"
#include "visual/BackendVisualEnsure.h"

#include <algorithm>

namespace cloudsim::host
{
BackendVisualSyncEngine::BackendVisualSyncEngine(DocumentHost& host) : m_host(host) {}

void BackendVisualSyncEngine::markDirty(const std::string& backendId, VisualAspect aspects, VisualChangeReason reason)
{
	(void)reason;
	if (backendId.empty() || aspects == VisualAspect::None)
	{
		return;
	}
	auto it = m_dirty.find(backendId);
	if (it == m_dirty.end())
	{
		m_dirty.emplace(backendId, aspects);
	}
	else
	{
		it->second = it->second | aspects;
	}
	if (m_kinematicsBatchDepth > 0 && hasVisualAspect(aspects, VisualAspect::Transform))
	{
		m_batchTransformIds.insert(backendId);
	}
	if (m_kinematicsBatchDepth > 0)
	{
		return;
	}
}

void BackendVisualSyncEngine::clear()
{
	m_dirty.clear();
	m_batchTransformIds.clear();
}

void BackendVisualSyncEngine::beginKinematicsBatch()
{
	++m_kinematicsBatchDepth;
	m_deferFollowSolveUntilBatchEnd = true;
}

void BackendVisualSyncEngine::endKinematicsBatch()
{
	if (m_kinematicsBatchDepth <= 0)
	{
		return;
	}
	--m_kinematicsBatchDepth;
	if (m_kinematicsBatchDepth > 0)
	{
		return;
	}
	m_deferFollowSolveUntilBatchEnd = false;
	std::vector<std::string> ordered(m_batchTransformIds.begin(), m_batchTransformIds.end());
	m_batchTransformIds.clear();
	(void)flushTransform(ordered);
}

bool BackendVisualSyncEngine::takePendingFollowSolveAfterBatch() const
{
	return m_pendingFollowSolveAfterBatch;
}

void BackendVisualSyncEngine::setPendingFollowSolveAfterBatch(const bool pending)
{
	m_pendingFollowSolveAfterBatch = pending;
}

std::vector<std::string> BackendVisualSyncEngine::resolveTransformFlushOrder(
	const std::vector<std::string>& hintOrder) const
{
	if (!hintOrder.empty())
	{
		return hintOrder;
	}
	std::vector<std::string> ordered;
	BackendDataManager& mgr = m_host.backend();
	const std::vector<std::string> topo = mgr.topoOrder();
	ordered.reserve(m_dirty.size());
	for (const std::string& id : topo)
	{
		const auto it = m_dirty.find(id);
		if (it == m_dirty.end() || !hasVisualAspect(it->second, VisualAspect::Transform))
		{
			continue;
		}
		ordered.push_back(id);
	}
	for (const auto& kv : m_dirty)
	{
		if (!hasVisualAspect(kv.second, VisualAspect::Transform))
		{
			continue;
		}
		if (std::find(ordered.begin(), ordered.end(), kv.first) == ordered.end())
		{
			ordered.push_back(kv.first);
		}
	}
	return ordered;
}

bool BackendVisualSyncEngine::flushTransformForId(const std::string& backendId)
{
	OsgWidget* osg = osgWidgetFrom(m_host);
	if (!osg)
	{
		return false;
	}
	// TCP 示教拖动不算对象 gizmo：须照常 flush 连杆 mesh，否则 Follow 读到陈旧父级
	if (osg->isTransformGizmoDragging() && !osg->isTcpDragTeachActive() && osg->activeBackendId() == backendId)
	{
		osg->requestRedraw();
		return true;
	}
	return osg->applyWorldMatrixToOsg(backendId, m_host.backend());
}

bool BackendVisualSyncEngine::flushAppearanceForId(const std::string& backendId)
{
	OsgWidget* osg = osgWidgetFrom(m_host);
	const auto obj = m_host.backend().getData(backendId);
	if (!osg || !obj)
	{
		return false;
	}
	const BackendColor color = obj->color();
	osg->applyColorToBackendObject(backendId, osg::Vec4(color.r, color.g, color.b, color.a));
	return true;
}

bool BackendVisualSyncEngine::flushVisibilityForId(const std::string& backendId)
{
	OsgWidget* osg = osgWidgetFrom(m_host);
	const auto obj = m_host.backend().getData(backendId);
	if (!osg || !obj)
	{
		return false;
	}
	osg->setBackendObjectVisible(backendId, obj->isVisible());
	return true;
}

bool BackendVisualSyncEngine::flushGeometryForId(const std::string& backendId)
{
	const auto obj = m_host.backend().getData(backendId);
	if (!obj)
	{
		return false;
	}
	const std::uint64_t rev = obj->geometryRevision();
	const auto revIt = m_lastSyncedGeometryRevision.find(backendId);
	if (revIt != m_lastSyncedGeometryRevision.end() && revIt->second == rev)
	{
		return true;
	}
	EnsureVisualOptions opts;
	(void)ensureVisual(m_host, backendId, EnsureVisualPolicy::FullRebuild, opts);
	m_lastSyncedGeometryRevision[backendId] = rev;
	return true;
}

bool BackendVisualSyncEngine::flushTransform(const std::vector<std::string>& orderedIds)
{
	const std::vector<std::string> order = resolveTransformFlushOrder(orderedIds);
	bool any = false;
	for (const std::string& id : order)
	{
		auto it = m_dirty.find(id);
		if (it == m_dirty.end() || !hasVisualAspect(it->second, VisualAspect::Transform))
		{
			continue;
		}
		any = flushTransformForId(id) || any;
		it->second = static_cast<VisualAspect>(static_cast<std::uint32_t>(it->second) &
											   ~static_cast<std::uint32_t>(VisualAspect::Transform));
		if (it->second == VisualAspect::None)
		{
			m_dirty.erase(it);
		}
	}
	if (OsgWidget* osg = osgWidgetFrom(m_host))
	{
		osg->requestRedraw();
	}
	return any;
}

bool BackendVisualSyncEngine::flush(FlushPolicy policy)
{
	(void)policy;
	std::vector<std::string> transformIds;
	for (const auto& kv : m_dirty)
	{
		if (hasVisualAspect(kv.second, VisualAspect::Transform))
		{
			transformIds.push_back(kv.first);
		}
	}
	(void)flushTransform(transformIds);

	for (auto it = m_dirty.begin(); it != m_dirty.end();)
	{
		const std::string id = it->first;
		VisualAspect remaining = it->second;
		if (hasVisualAspect(remaining, VisualAspect::Appearance))
		{
			(void)flushAppearanceForId(id);
			remaining = static_cast<VisualAspect>(static_cast<std::uint32_t>(remaining) &
												  ~static_cast<std::uint32_t>(VisualAspect::Appearance));
		}
		if (hasVisualAspect(remaining, VisualAspect::Visibility))
		{
			(void)flushVisibilityForId(id);
			remaining = static_cast<VisualAspect>(static_cast<std::uint32_t>(remaining) &
												  ~static_cast<std::uint32_t>(VisualAspect::Visibility));
		}
		if (hasVisualAspect(remaining, VisualAspect::Geometry))
		{
			(void)flushGeometryForId(id);
			remaining = static_cast<VisualAspect>(static_cast<std::uint32_t>(remaining) &
												  ~static_cast<std::uint32_t>(VisualAspect::Geometry));
		}
		if (remaining == VisualAspect::None)
		{
			it = m_dirty.erase(it);
		}
		else
		{
			it->second = remaining;
			++it;
		}
	}
	return true;
}

} // namespace cloudsim::host
