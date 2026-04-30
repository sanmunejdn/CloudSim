#include "MainWindowObjectGraph.h"

#include <QSet>

#include "BackendDataBase.h"

namespace
{

bool wouldCreateCycle(
	const QString& childId,
	const QString& parentId,
	const QHash<QString, MainWindowObjectGraph::Node>& nodes)
{
	QString cursor = parentId;
	QSet<QString> visited;
	while (!cursor.isEmpty())
	{
		if (cursor == childId)
		{
			return true;
		}
		if (visited.contains(cursor))
		{
			return true;
		}
		visited.insert(cursor);
		auto it = nodes.constFind(cursor);
		if (it == nodes.constEnd())
		{
			return false;
		}
		cursor = it->parentId;
	}
	return false;
}

} // namespace

MainWindowObjectGraph MainWindowObjectGraph::build(
	const std::vector<std::shared_ptr<BackendDataBase>>& objects,
	const QMap<QString, QString>& parentById)
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
		node.parentId = parentById.value(id);
		node.data = data;
		graph.m_nodes.insert(id, std::move(node));
		graph.m_nodeOrder.append(id);
	}

	for (const QString& id : graph.m_nodeOrder)
	{
		auto it = graph.m_nodes.find(id);
		if (it == graph.m_nodes.end())
		{
			continue;
		}
		QString parentId = it->parentId;
		if (parentId.isEmpty() || parentId == id || !graph.m_nodes.contains(parentId))
		{
			it->parentId.clear();
			continue;
		}
		if (wouldCreateCycle(id, parentId, graph.m_nodes))
		{
			it->parentId.clear();
			continue;
		}
		graph.m_nodes[parentId].childIds.append(id);
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
	queue.append(rootId);
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
			queue.append(childId);
		}
	}
	return out;
}
