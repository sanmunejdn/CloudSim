#pragma once

#include "osgwidgetcore_global.h"

#include <string>
#include <unordered_map>

#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/observer_ptr>

/// Centralized index for backend object id <-> scene root node bindings.
/// Picking resolves backend identity through this index instead of ad-hoc maps.
class OSGWIDGETCORE_EXPORT BackendVisualBindingIndex
{
public:
	void bindBackendRoot(const std::string& backendId, osg::Node* rootNode);
	void unbindBackend(const std::string& backendId);
	void clear();

	/// Resolve backend id for a picked node path; returns false when no bound backend root is present.
	bool resolveBackendIdFromNodePath(const osg::NodePath& path, std::string& outBackendId) const;

private:
	std::unordered_map<std::string, osg::observer_ptr<osg::Node>> m_backendToRoot;
	std::unordered_map<const osg::Node*, std::string> m_rootToBackend;
};

