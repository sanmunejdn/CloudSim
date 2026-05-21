#if defined(_WIN32)
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include "BackendVisualBindingIndex.h"

#include "BackendIdUserData.h"

void BackendVisualBindingIndex::bindBackendRoot(const std::string& backendId, osg::Node* rootNode)
{
	if (backendId.empty() || !rootNode)
	{
		return;
	}
	unbindBackend(backendId);
	m_backendToRoot[backendId] = rootNode;
	m_rootToBackend[rootNode] = backendId;
}

void BackendVisualBindingIndex::unbindBackend(const std::string& backendId)
{
	if (backendId.empty())
	{
		return;
	}
	const auto it = m_backendToRoot.find(backendId);
	if (it == m_backendToRoot.end())
	{
		return;
	}
	if (it->second.valid())
	{
		m_rootToBackend.erase(it->second.get());
	}
	m_backendToRoot.erase(it);
}

void BackendVisualBindingIndex::clear()
{
	m_backendToRoot.clear();
	m_rootToBackend.clear();
}

bool BackendVisualBindingIndex::resolveBackendIdFromNodePath(const osg::NodePath& path, std::string& outBackendId) const
{
	for (auto it = path.rbegin(); it != path.rend(); ++it)
	{
		const osg::Node* node = *it;
		if (!node)
		{
			continue;
		}
		const auto idIt = m_rootToBackend.find(node);
		if (idIt != m_rootToBackend.end())
		{
			outBackendId = idIt->second;
			return true;
		}
	}

	// Compatibility bridge: allow resolving from node user data if caller provided only scene path.
	const BackendIdUserData* userId = BackendIdUserData::findInNodePath(path);
	if (!userId)
	{
		return false;
	}
	const std::string& id = userId->backendId();
	if (id.empty())
	{
		return false;
	}
	outBackendId = id;
	return true;
}

