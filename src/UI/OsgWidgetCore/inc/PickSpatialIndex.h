#ifndef OSGWIDGETCORE_PICKSPATIALINDEX_H
#define OSGWIDGETCORE_PICKSPATIALINDEX_H

/// @file PickSpatialIndex.h
/// @brief 点云拾取空间索引：绑定 Visual 时构建，选中切换时只读引用

#include "osgwidgetcore_global.h"

#include <cstdint>
#include <vector>

#include <osg/Node>
#include <osg/Vec3f>

/// 点云拾取空间索引：绑定 Visual 时构建，选中切换时只读引用
class OSGWIDGETCORE_EXPORT PickSpatialIndex
{
public:
	void buildFromNode(osg::Node* node);
	void clear();

	bool empty() const { return m_pointsLocal.empty(); }
	std::size_t pointCount() const { return m_pointsLocal.size(); }
	std::uint64_t generation() const { return m_generation; }

	const std::vector<osg::Vec3f>& pointsLocal() const { return m_pointsLocal; }
	const std::vector<osg::Vec3f>& pointsCenteredLocal() const { return m_pointsCenteredLocal; }
	bool hasKdTree() const { return m_kdRoot >= 0; }

	void nearestCandidates(const osg::Vec3f& queryLocalCentered, int k, std::vector<int>& outIndices) const;

private:
	struct KdNode
	{
		int pointIndex = -1;
		int axis = 0;
		int left = -1;
		int right = -1;
	};

	void rebuildKdTree();
	int buildKdNode(std::vector<int>& indices, int begin, int end, int depth);

	std::vector<osg::Vec3f> m_pointsLocal;
	std::vector<osg::Vec3f> m_pointsCenteredLocal;
	std::vector<KdNode> m_kdNodes;
	int m_kdRoot = -1;
	std::uint64_t m_generation = 0;
};

#endif // OSGWIDGETCORE_PICKSPATIALINDEX_H
