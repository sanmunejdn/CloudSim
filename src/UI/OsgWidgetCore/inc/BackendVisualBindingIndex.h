#pragma once

#include "osgwidgetcore_global.h"

#include <string>
#include <unordered_map>

#include <osg/Node>
#include <osg/NodeVisitor>
#include <osg/observer_ptr>

/// 后端 id 与场景根节点绑定索引
/// 拾取经此索引解析后端 id
class OSGWIDGETCORE_EXPORT BackendVisualBindingIndex
{
public:
	void bindBackendRoot(const std::string& backendId, osg::Node* rootNode);
	void unbindBackend(const std::string& backendId);
	void clear();

	/// 解析拾取路径的后端 id；无绑定根则 false
	bool resolveBackendIdFromNodePath(const osg::NodePath& path, std::string& outBackendId) const;

private:
	std::unordered_map<std::string, osg::observer_ptr<osg::Node>> m_backendToRoot;
	std::unordered_map<const osg::Node*, std::string> m_rootToBackend;
};

