#include "MainWindowObjectGraph.h"

#include <algorithm>
#include <QSet>

#include "BackendDataBase.h"

namespace
{

bool wouldCreateCycle(
	const QString& childId,
	const QString& parentId,
	const QHash<QString, MainWindowObjectGraph::Node>& nodes)
{
	QVector<QString> queue;
	QSet<QString> visited;
	queue.append(childId);
	visited.insert(childId);
	for (int i = 0; i < queue.size(); ++i)
	{
		const QString cursor = queue[i];
		if (cursor == parentId)
		{
			return true;
		}
		auto it = nodes.constFind(cursor);
		if (it == nodes.constEnd())
		{
			continue;
		}
		for (const QString& next : it->childIds)
		{
			if (!visited.contains(next))
			{
				visited.insert(next);
				queue.append(next);
			}
		}
	}
	return false;
}

void appendUniqueId(QVector<QString>& out, const QString& id)
{
	if (!out.contains(id))
	{
		out.append(id);
	}
}

} // namespace

MainWindowObjectGraph MainWindowObjectGraph::build(
	const std::vector<std::shared_ptr<BackendDataBase>>& objects,
	const std::vector<std::pair<QString, QString>>& edges)
{
	MainWindowObjectGraph graph;
	graph.m_nodes.reserve(static_cast<int>(objects.size()));
	graph.m_nodeOrder.reserve(static_cast<int>(objects.size()));

	for (const std::shared_ptr<BackendDataBase>& data : objects)
	{
		if (!data)
		{
			continue;
		}
		const QString id = QString::fromStdString(data->id());
		if (id.isEmpty() || graph.m_nodes.contains(id))
		{
			continue;
		}
		Node node;
		node.id = id;
		node.data = data;
		graph.m_nodes.insert(id, std::move(node));
		graph.m_nodeOrder.append(id);
	}

	for (const auto& edge : edges)
	{
		const QString parentId = edge.first;
		const QString childId = edge.second;
		if (parentId.isEmpty() || childId.isEmpty() || parentId == childId)
		{
			continue;
		}
		if (!graph.m_nodes.contains(parentId) || !graph.m_nodes.contains(childId))
		{
			continue;
		}
		auto childIt = graph.m_nodes.find(childId);
		if (childIt == graph.m_nodes.end())
		{
			continue;
		}
		appendUniqueId(graph.m_nodes[parentId].childIds, childId);
		appendUniqueId(childIt->parentIds, parentId);
		if (wouldCreateCycle(childId, parentId, graph.m_nodes))
		{
			graph.m_nodes[parentId].childIds.removeAll(childId);
			childIt->parentIds.removeAll(parentId);
		}
	}

	for (auto it = graph.m_nodes.begin(); it != graph.m_nodes.end(); ++it)
	{
		std::sort(it->parentIds.begin(), it->parentIds.end());
		std::sort(it->childIds.begin(), it->childIds.end());
		it->primaryParentId = it->parentIds.isEmpty() ? QString() : it->parentIds.front();
	}

	return graph;
}

const MainWindowObjectGraph::Node* MainWindowObjectGraph::node(const QString& id) const
{
	auto it = m_nodes.constFind(id);
	if (it == m_nodes.constEnd())
	{
		return nullptr;
	}
	return &it.value();
}

QVector<QString> MainWindowObjectGraph::subtreeIds(const QString& rootId) const
{
	QVector<QString> out;
	if (rootId.isEmpty() || !m_nodes.contains(rootId))
	{
		return out;
	}
	QVector<QString> queue;
	QSet<QString> visited;
	queue.append(rootId);
	visited.insert(rootId);
	for (int index = 0; index < queue.size(); ++index)
	{
		const QString id = queue[index];
		out.append(id);
		const Node* const current = node(id);
		if (!current)
		{
			continue;
		}
		for (const QString& childId : current->childIds)
		{
			if (!visited.contains(childId))
			{
				visited.insert(childId);
				queue.append(childId);
			}
		}
	}
	return out;
}
