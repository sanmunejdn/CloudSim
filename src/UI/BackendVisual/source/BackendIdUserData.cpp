/// @file BackendIdUserData.cpp
/// @brief 后端 ID 用户数据

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

#include <osg/Group>
#include <osg/PositionAttitudeTransform>

BackendIdUserData::BackendIdUserData(std::string id) : m_id(std::move(id)) {}

void BackendIdUserData::attach(osg::Node* root, const std::string& backendId)
{
	if (!root || backendId.empty())
	{
		return;
	}
	auto* meta = new BackendIdUserData(backendId);
	meta->m_pickDomain = BackendPickDomain::Mesh;
	root->setUserData(meta);
}

void BackendIdUserData::attachBrep(osg::Node* root, const std::string& backendId, const geoalgo::ShapeHandle& shape)
{
	if (!root || backendId.empty() || shape.isNull())
	{
		return;
	}
	auto* meta = new BackendIdUserData(backendId);
	meta->m_pickDomain = BackendPickDomain::Brep;
	meta->m_brepShape = shape.clone();
	root->setUserData(meta);
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

osg::Node* backendVisualResolvePickNode(osg::Group* outerBranchRoot)
{
	if (!outerBranchRoot || outerBranchRoot->getNumChildren() == 0)
	{
		return nullptr;
	}
	auto* inner = dynamic_cast<osg::PositionAttitudeTransform*>(outerBranchRoot->getChild(0));
	if (!inner || inner->getNumChildren() == 0)
	{
		return nullptr;
	}
	return inner->getChild(0);
}
