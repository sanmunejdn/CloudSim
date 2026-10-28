#ifndef OSGWIDGETCORE_BACKENDPICKINDEXREGISTRY_H
#define OSGWIDGETCORE_BACKENDPICKINDEXREGISTRY_H

/// @file BackendPickIndexRegistry.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief backendId → 拾取索引（随 bindBackendVisualRoot 构建）

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
class ShapeHandle;
struct BrepImportArtifacts;
}

struct OSGWIDGETCORE_EXPORT BackendPickBundle
{
	PickSpatialIndex pointIndex;
	BrepPickIndex brepIndex;
	/// bind 时持有；Phase2 边折线懒构建后 sync 进 brepIndex
	std::shared_ptr<geoalgo::BrepImportArtifacts> brepArtifacts;
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
	/// 首次边拾取：ensure Phase2 并写入 brepIndex（导入时不做，避免装配体卡死）
	bool ensureBrepEdgePickReady(const std::string& backendId, const geoalgo::ShapeHandle& shape);

private:
	std::unordered_map<std::string, BackendPickBundle> m_bundles;
	std::uint64_t m_nextGeneration = 1;
};

#endif // OSGWIDGETCORE_BACKENDPICKINDEXREGISTRY_H
