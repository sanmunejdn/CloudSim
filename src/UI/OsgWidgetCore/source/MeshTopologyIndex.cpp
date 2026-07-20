/// @file MeshTopologyIndex.cpp
/// @brief MeshTopologyIndex 实现

#include "pch.h"

#include "MeshTopologyIndex.h"

#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Matrix>
#include <osg/NodeVisitor>
#include <osg/PrimitiveSet>
#include <osg/Vec3d>

void MeshTopologyIndex::clear()
{
	m_triangleSoupLocal.clear();
	++m_generation;
}

void MeshTopologyIndex::buildFromNode(osg::Node* node)
{
	clear();
	if (!node)
	{
		return;
	}

	struct TriCollectVisitor : public osg::NodeVisitor
	{
		explicit TriCollectVisitor(std::vector<osg::Vec3>& out) : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), soup(out) {}

		void apply(osg::Geode& geode) override
		{
			const osg::Matrixd localToRoot = osg::computeLocalToWorld(getNodePath());
			for (unsigned int di = 0; di < geode.getNumDrawables(); ++di)
			{
				const osg::Geometry* geom = geode.getDrawable(di) ? geode.getDrawable(di)->asGeometry() : nullptr;
				if (!geom || !geom->getVertexArray())
				{
					continue;
				}
				const osg::Vec3Array* vertices = dynamic_cast<const osg::Vec3Array*>(geom->getVertexArray());
				if (!vertices)
				{
					continue;
				}
				for (unsigned int pi = 0; pi < geom->getNumPrimitiveSets(); ++pi)
				{
					const osg::PrimitiveSet* ps = geom->getPrimitiveSet(pi);
					if (!ps)
					{
						continue;
					}
					if (ps->getMode() != osg::PrimitiveSet::TRIANGLES)
					{
						continue;
					}
					for (unsigned int ti = 0; ti + 2 < ps->getNumIndices(); ti += 3)
					{
						for (int k = 0; k < 3; ++k)
						{
							const unsigned int idx = ps->index(ti + static_cast<unsigned int>(k));
							if (idx >= vertices->size())
							{
								continue;
							}
							const osg::Vec3& v = (*vertices)[idx];
							const osg::Vec3d p = osg::Vec3d(v.x(), v.y(), v.z()) * localToRoot;
							soup.emplace_back(static_cast<float>(p.x()), static_cast<float>(p.y()),
											  static_cast<float>(p.z()));
						}
					}
				}
			}
			traverse(geode);
		}

		std::vector<osg::Vec3>& soup;
	};

	TriCollectVisitor collector(m_triangleSoupLocal);
	node->accept(collector);
}
