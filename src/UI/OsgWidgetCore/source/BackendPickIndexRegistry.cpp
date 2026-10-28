/// @file BackendPickIndexRegistry.cpp
/// @brief BackendPickIndexRegistry 实现

#include "pch.h"

#include "BackendPickIndexRegistry.h"

#include "BackendIdUserData.h"
#include "BackendPickDomain.h"

#include <chrono>

#include <BrepImportArtifacts.h>
#include <RunLogger.h>
#include <osg/Group>
#include <osg/Node>

namespace
{
osg::Node* resolvePickNode(osg::Node* rootNode)
{
	if (auto* group = dynamic_cast<osg::Group*>(rootNode))
	{
		return backendVisualResolvePickNode(group);
	}
	return rootNode;
}

void bindBrepPickIndex(BrepPickIndex& brepIndex, const BackendIdUserData* meta,
					   const std::shared_ptr<geoalgo::BrepImportArtifacts>& brepArtifacts)
{
	brepIndex = BrepPickIndex{};
	if (!meta || !meta->hasBrepShape())
	{
		return;
	}
	std::string err;
	if (brepArtifacts)
	{
		// 边离散放到首次边拾取；导入时做会把装配体拖死
		(void)brepIndex.buildFromArtifacts(*brepArtifacts, &err);
		return;
	}
	(void)brepIndex.build(meta->brepShape(), &err);
}

} // namespace

void BackendPickIndexRegistry::bindBackendRoot(const std::string& backendId, osg::Node* rootNode)
{
	bindBackendRoot(backendId, rootNode, {});
}

void BackendPickIndexRegistry::bindBackendRoot(const std::string& backendId, osg::Node* rootNode,
											   const std::shared_ptr<geoalgo::BrepImportArtifacts>& brepArtifacts)
{
	if (backendId.empty() || !rootNode)
	{
		return;
	}
	BackendPickBundle& bundle = m_bundles[backendId];
	bundle.generation = m_nextGeneration++;
	bundle.brepArtifacts = brepArtifacts;
	const BackendIdUserData* meta = dynamic_cast<const BackendIdUserData*>(rootNode->getUserData());
	const bool isBrep = meta && meta->pickDomain() == BackendPickDomain::Brep;
	const auto t0 = std::chrono::steady_clock::now();
	bundle.pointIndex.clear();
	if (!isBrep)
	{
		osg::Node* pickNode = resolvePickNode(rootNode);
		if (!pickNode)
		{
			pickNode = rootNode;
		}
		bundle.pointIndex.buildFromNode(pickNode);
	}
	bindBrepPickIndex(bundle.brepIndex, meta, brepArtifacts);
	const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - t0);
	RunLogger::info("[Import] bindBackendPickIndex " + std::to_string(ms.count()) + " ms, id=" + backendId +
					(isBrep ? ", brep" : ""));
}

bool BackendPickIndexRegistry::ensureBrepEdgePickReady(const std::string& backendId, const geoalgo::ShapeHandle& shape)
{
	if (backendId.empty() || shape.isNull())
	{
		return false;
	}
	const auto it = m_bundles.find(backendId);
	if (it == m_bundles.end())
	{
		return false;
	}
	BackendPickBundle& bundle = it->second;
	if (bundle.brepIndex.edgesReady())
	{
		return true;
	}
	if (!bundle.brepArtifacts)
	{
		bundle.brepArtifacts = geoalgo::getOrBuildBrepImportArtifacts(shape, nullptr);
	}
	if (!bundle.brepArtifacts)
	{
		return false;
	}
	// 线框路径可能已写好 Phase2，索引仍是 bind 时的空拷贝 → 先 sync
	if (bundle.brepArtifacts->pickReady.load(std::memory_order_acquire) &&
		bundle.brepIndex.applyEdgePickArtifacts(*bundle.brepArtifacts))
	{
		return true;
	}
	std::string err;
	if (!geoalgo::ensureBrepImportPickArtifacts(shape, *bundle.brepArtifacts, &err))
	{
		RunLogger::warn("[Pick] ensureBrepEdgePickReady Phase2 failed id=" + backendId + " err=" + err);
		return false;
	}
	if (!bundle.brepIndex.applyEdgePickArtifacts(*bundle.brepArtifacts))
	{
		RunLogger::warn("[Pick] ensureBrepEdgePickReady empty edge polylines id=" + backendId);
		return false;
	}
	RunLogger::info("[Pick] ensureBrepEdgePickReady ok id=" + backendId +
					" edges=" + std::to_string(bundle.brepArtifacts->edgePolylines.size()));
	return true;
}

void BackendPickIndexRegistry::unbindBackend(const std::string& backendId)
{
	m_bundles.erase(backendId);
}

void BackendPickIndexRegistry::clear()
{
	m_bundles.clear();
	m_nextGeneration = 1;
}

const BackendPickBundle* BackendPickIndexRegistry::find(const std::string& backendId) const
{
	const auto it = m_bundles.find(backendId);
	if (it == m_bundles.end())
	{
		return nullptr;
	}
	return &it->second;
}

void BackendPickIndexRegistry::bumpGeneration(const std::string& backendId)
{
	const auto it = m_bundles.find(backendId);
	if (it == m_bundles.end())
	{
		return;
	}
	it->second.generation = m_nextGeneration++;
}
