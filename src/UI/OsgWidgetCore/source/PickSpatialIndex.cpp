#include "pch.h"
#include "PickSpatialIndex.h"

#include <algorithm>
#include <functional>
#include <limits>
#include <queue>

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Matrix>
#include <osg/NodeVisitor>
#include <osg/Vec3d>

void PickSpatialIndex::clear()
{
	m_pointsLocal.clear();
	m_pointsCenteredLocal.clear();
	m_kdNodes.clear();
	m_kdRoot = -1;
	++m_generation;
}

void PickSpatialIndex::buildFromNode(osg::Node* node)
{
	clear();
	if (!node)
	{
		return;
	}

	struct PointCollectVisitor : public osg::NodeVisitor
	{
		explicit PointCollectVisitor(std::vector<osg::Vec3f>& out)
			: osg::NodeVisitor(TRAVERSE_ALL_CHILDREN)
			, points(out)
		{
		}

		void apply(osg::Geode& geode) override
		{
			const osg::Matrixd localToRoot = osg::computeLocalToWorld(getNodePath());
			for (unsigned int i = 0; i < geode.getNumDrawables(); ++i)
			{
				osg::Geometry* geom = geode.getDrawable(i) ? geode.getDrawable(i)->asGeometry() : nullptr;
				if (!geom || !geom->getVertexArray())
				{
					continue;
				}
				const osg::Vec3Array* vertices = dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray());
				if (vertices)
				{
					points.reserve(points.size() + vertices->size());
					for (const osg::Vec3& v : *vertices)
					{
						const osg::Vec3d p = osg::Vec3d(v.x(), v.y(), v.z()) * localToRoot;
						points.emplace_back(
							static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
					}
					continue;
				}
				const osg::Vec3dArray* verticesD = dynamic_cast<const osg::Vec3dArray*>(geom->getVertexArray());
				if (verticesD)
				{
					points.reserve(points.size() + verticesD->size());
					for (const osg::Vec3d& v : *verticesD)
					{
						const osg::Vec3d p = v * localToRoot;
						points.emplace_back(
							static_cast<float>(p.x()), static_cast<float>(p.y()), static_cast<float>(p.z()));
					}
				}
			}
			traverse(geode);
		}

		std::vector<osg::Vec3f>& points;
	};

	PointCollectVisitor collector(m_pointsLocal);
	node->accept(collector);
	if (m_pointsLocal.empty())
	{
		return;
	}

	osg::Vec3f center(0.0f, 0.0f, 0.0f);
	for (const osg::Vec3f& p : m_pointsLocal)
	{
		center += p;
	}
	center /= static_cast<float>(m_pointsLocal.size());

	m_pointsCenteredLocal.reserve(m_pointsLocal.size());
	for (const osg::Vec3f& p : m_pointsLocal)
	{
		m_pointsCenteredLocal.push_back(p - center);
	}
	rebuildKdTree();
}

void PickSpatialIndex::rebuildKdTree()
{
	m_kdNodes.clear();
	m_kdRoot = -1;
	if (m_pointsCenteredLocal.empty())
	{
		return;
	}
	static const std::size_t kMaxKdPoints = 1500000;
	if (m_pointsCenteredLocal.size() > kMaxKdPoints)
	{
		return;
	}

	std::vector<int> indices(m_pointsCenteredLocal.size());
	for (std::size_t i = 0; i < indices.size(); ++i)
	{
		indices[i] = static_cast<int>(i);
	}
	m_kdNodes.reserve(m_pointsCenteredLocal.size());
	m_kdRoot = buildKdNode(indices, 0, static_cast<int>(indices.size()), 0);
}

int PickSpatialIndex::buildKdNode(std::vector<int>& indices, int begin, int end, int depth)
{
	if (begin >= end)
	{
		return -1;
	}
	const int axis = depth % 3;
	const int mid = begin + (end - begin) / 2;
	std::nth_element(indices.begin() + begin, indices.begin() + mid, indices.begin() + end,
		[this, axis](int a, int b) {
			const osg::Vec3f& pa = m_pointsCenteredLocal[static_cast<std::size_t>(a)];
			const osg::Vec3f& pb = m_pointsCenteredLocal[static_cast<std::size_t>(b)];
			if (axis == 0)
			{
				return pa.x() < pb.x();
			}
			if (axis == 1)
			{
				return pa.y() < pb.y();
			}
			return pa.z() < pb.z();
		});

	const int nodeIndex = static_cast<int>(m_kdNodes.size());
	m_kdNodes.emplace_back();
	m_kdNodes[static_cast<std::size_t>(nodeIndex)].pointIndex = indices[static_cast<std::size_t>(mid)];
	m_kdNodes[static_cast<std::size_t>(nodeIndex)].axis = axis;
	m_kdNodes[static_cast<std::size_t>(nodeIndex)].left = buildKdNode(indices, begin, mid, depth + 1);
	m_kdNodes[static_cast<std::size_t>(nodeIndex)].right = buildKdNode(indices, mid + 1, end, depth + 1);
	return nodeIndex;
}

void PickSpatialIndex::nearestCandidates(
	const osg::Vec3f& queryLocalCentered,
	int k,
	std::vector<int>& outIndices) const
{
	outIndices.clear();
	if (m_kdRoot < 0 || m_kdNodes.empty() || m_pointsCenteredLocal.empty() || k <= 0)
	{
		return;
	}

	using DistIndex = std::pair<float, int>;
	std::priority_queue<DistIndex> best;

	std::function<void(int)> dfs = [&](int nodeIndex) {
		if (nodeIndex < 0)
		{
			return;
		}
		const KdNode& node = m_kdNodes[static_cast<std::size_t>(nodeIndex)];
		const osg::Vec3f& p = m_pointsCenteredLocal[static_cast<std::size_t>(node.pointIndex)];
		const osg::Vec3f d = p - queryLocalCentered;
		const float dist2 = d.length2();

		if (static_cast<int>(best.size()) < k)
		{
			best.push({ dist2, node.pointIndex });
		}
		else if (dist2 < best.top().first)
		{
			best.pop();
			best.push({ dist2, node.pointIndex });
		}

		float diff = 0.0f;
		if (node.axis == 0)
		{
			diff = queryLocalCentered.x() - p.x();
		}
		else if (node.axis == 1)
		{
			diff = queryLocalCentered.y() - p.y();
		}
		else
		{
			diff = queryLocalCentered.z() - p.z();
		}

		const int nearChild = diff <= 0.0f ? node.left : node.right;
		const int farChild = diff <= 0.0f ? node.right : node.left;
		dfs(nearChild);
		const float worst = (static_cast<int>(best.size()) < k)
			? (std::numeric_limits<float>::max)()
			: best.top().first;
		if (diff * diff < worst)
		{
			dfs(farChild);
		}
	};
	dfs(m_kdRoot);

	outIndices.reserve(best.size());
	while (!best.empty())
	{
		outIndices.push_back(best.top().second);
		best.pop();
	}
}
