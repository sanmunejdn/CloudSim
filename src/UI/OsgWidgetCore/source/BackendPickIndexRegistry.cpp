#include "pch.h"
#include "BackendPickIndexRegistry.h"

#include "BackendIdUserData.h"

#include <osg/Group>
#include <osg/Node>

namespace {

osg::Node* resolvePickNode(osg::Node* rootNode)
{
	if (auto* group = dynamic_cast<osg::Group*>(rootNode))
	{
		return backendVisualResolvePickNode(group);
	}
	return rootNode;
}

} // namespace

void BackendPickIndexRegistry::bindBackendRoot(const std::string& backendId, osg::Node* rootNode)
{
	if (backendId.empty() || !rootNode)
	{
		return;
	}
	BackendPickBundle& bundle = m_bundles[backendId];
	bundle.generation = m_nextGeneration++;
	osg::Node* pickNode = resolvePickNode(rootNode);
	if (!pickNode)
	{
		pickNode = rootNode;
	}
	bundle.pointIndex.buildFromNode(pickNode);
	bundle.meshIndex.buildFromNode(pickNode);
}

void BackendPickIndexRegistry::unbindBackend(const std::string& backendId)
{
	m_bundles.erase(backendId);
}

void BackendPickIndexRegistry::clear()
{
	m_bundles.clear();
	m_nextGeneration = 1;
}

const BackendPickBundle* BackendPickIndexRegistry::find(const std::string& backendId) const
{
	const auto it = m_bundles.find(backendId);
	if (it == m_bundles.end())
	{
		return nullptr;
	}
	return &it->second;
}

void BackendPickIndexRegistry::bumpGeneration(const std::string& backendId)
{
	const auto it = m_bundles.find(backendId);
	if (it == m_bundles.end())
	{
		return;
	}
	it->second.generation = m_nextGeneration++;
}
