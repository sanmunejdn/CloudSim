#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendIdUserData.h"

#include <osg/PositionAttitudeTransform>

BackendIdUserData::BackendIdUserData(std::string id)
	: m_id(std::move(id))
{
}

void BackendIdUserData::attach(osg::Node* root, const std::string& backendId)
{
	if (!root || backendId.empty())
	{
		return;
	}
	root->setUserData(new BackendIdUserData(backendId));
}

const BackendIdUserData* BackendIdUserData::findInNodePath(const osg::NodePath& path)
{
	for (auto it = path.rbegin(); it != path.rend(); ++it)
	{
		const osg::Node* n = *it;
		if (!n)
		{
			continue;
		}
		const osg::Referenced* ud = n->getUserData();
		if (!ud)
		{
			continue;
		}
		if (auto* bid = dynamic_cast<const BackendIdUserData*>(ud))
		{
			return bid;
		}
	}
	return nullptr;
}

osg::Node* backendVisualResolvePickNode(osg::PositionAttitudeTransform* outerPat)
{
	if (!outerPat || outerPat->getNumChildren() == 0)
	{
		return nullptr;
	}
	auto* inner = dynamic_cast<osg::PositionAttitudeTransform*>(outerPat->getChild(0));
	if (!inner || inner->getNumChildren() == 0)
	{
		return nullptr;
	}
	return inner->getChild(0);
}
