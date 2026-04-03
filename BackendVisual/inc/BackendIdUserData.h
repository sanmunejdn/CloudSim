#pragma once

#include "backendvisual_global.h"

#include <osg/Node>
#include <osg/Referenced>
#include <string>

namespace osg {
class PositionAttitudeTransform;
}

/// Attached to backend root scene nodes so picking can resolve \ref backendId() without a parallel map.
class BACKENDVISUAL_EXPORT BackendIdUserData : public osg::Referenced
{
public:
	explicit BackendIdUserData(std::string id);

	const std::string& backendId() const { return m_id; }

	static void attach(osg::Node* root, const std::string& backendId);
	/// Walk \a path from leaf toward root; first node carrying BackendIdUserData wins.
	static const BackendIdUserData* findInNodePath(const osg::NodePath& path);

private:
	std::string m_id;
};

/// Same hierarchy convention as legacy OsgWidget upsert: outer PAT → inner PAT → geometry root.
BACKENDVISUAL_EXPORT osg::Node* backendVisualResolvePickNode(osg::PositionAttitudeTransform* outerPat);
