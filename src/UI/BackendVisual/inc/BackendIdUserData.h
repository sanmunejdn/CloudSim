#ifndef BACKENDVISUAL_BACKENDIDUSERDATA_H
#define BACKENDVISUAL_BACKENDIDUSERDATA_H

/// @file BackendIdUserData.h
/// @brief 挂后端根节点，拾取可解析 backendId，免平行 map

#include "backendvisual_global.h"

#include "BackendPickDomain.h"

#include <string>

#include <ShapeHandle.h>
#include <osg/Node>
#include <osg/Referenced>

namespace osg
{
class Group;
}

/// 挂后端根节点，拾取可解析 backendId，免平行 map
class BACKENDVISUAL_EXPORT BackendIdUserData : public osg::Referenced
{
public:
	explicit BackendIdUserData(std::string id);

	const std::string& backendId() const { return m_id; }
	BackendPickDomain pickDomain() const { return m_pickDomain; }
	const geoalgo::ShapeHandle& brepShape() const { return m_brepShape; }
	bool hasBrepShape() const { return m_pickDomain == BackendPickDomain::Brep && !m_brepShape.isNull(); }

	static void attach(osg::Node* root, const std::string& backendId);
	static void attachBrep(osg::Node* root, const std::string& backendId, const geoalgo::ShapeHandle& shape);
	/// 自叶向根遍历，首个 BackendIdUserData 生效
	static const BackendIdUserData* findInNodePath(const osg::NodePath& path);

private:
	std::string m_id;
	BackendPickDomain m_pickDomain = BackendPickDomain::Mesh;
	geoalgo::ShapeHandle m_brepShape;
};

/// 同 OsgWidget upsert：外层 MT → 内层 PAT → 几何根
BACKENDVISUAL_EXPORT osg::Node* backendVisualResolvePickNode(osg::Group* outerBranchRoot);

#endif // BACKENDVISUAL_BACKENDIDUSERDATA_H
