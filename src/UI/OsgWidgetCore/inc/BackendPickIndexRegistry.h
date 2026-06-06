#pragma once

#include "osgwidgetcore_global.h"
#include "BrepPickIndex.h"
#include "PickSpatialIndex.h"

#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>

#include <osg/Node>

namespace geoalgo
{
struct BrepImportArtifacts;
}

struct OSGWIDGETCORE_EXPORT BackendPickBundle
{
	PickSpatialIndex pointIndex;
	BrepPickIndex brepIndex;
	std::uint64_t generation = 0;
};

/// backendId → 拾取索引（随 bindBackendVisualRoot 构建）
class OSGWIDGETCORE_EXPORT BackendPickIndexRegistry
{
public:
	void bindBackendRoot(const std::string& backendId, osg::Node* rootNode);
	void bindBackendRoot(const std::string& backendId, osg::Node* rootNode,
		const std::shared_ptr<geoalgo::BrepImportArtifacts>& brepArtifacts);
	void unbindBackend(const std::string& backendId);
	void clear();

	const BackendPickBundle* find(const std::string& backendId) const;
	void bumpGeneration(const std::string& backendId);

private:
	std::unordered_map<std::string, BackendPickBundle> m_bundles;
	std::uint64_t m_nextGeneration = 1;
};
