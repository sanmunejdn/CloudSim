#ifndef OSGWIDGETCORE_MESHTOPOLOGYINDEX_H
#define OSGWIDGETCORE_MESHTOPOLOGYINDEX_H

/// @file MeshTopologyIndex.h
/// @brief 网格拓扑缓存：三角 soup（Visual 局部坐标），供面/边拾取 scope 与后续扩展

#include "osgwidgetcore_global.h"

#include <cstdint>
#include <vector>

#include <osg/Node>
#include <osg/Vec3>

/// 网格拓扑缓存：三角 soup（Visual 局部坐标），供面/边拾取 scope 与后续扩展
class OSGWIDGETCORE_EXPORT MeshTopologyIndex
{
public:
	void buildFromNode(osg::Node* node);
	void clear();

	bool empty() const { return m_triangleSoupLocal.empty(); }
	std::size_t triangleCount() const { return m_triangleSoupLocal.size() / 3U; }
	std::uint64_t generation() const { return m_generation; }
	const std::vector<osg::Vec3>& triangleSoupLocal() const { return m_triangleSoupLocal; }

private:
	std::vector<osg::Vec3> m_triangleSoupLocal;
	std::uint64_t m_generation = 0;
};

#endif // OSGWIDGETCORE_MESHTOPOLOGYINDEX_H
