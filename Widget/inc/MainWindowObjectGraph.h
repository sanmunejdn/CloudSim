#pragma once

#include <QHash>
#include <QString>
#include <QVector>
#include <memory>
#include <utility>
#include <vector>

class BackendDataBase;

/// MainWindow 侧后端对象关系图（只读视图），提供父子结构与根节点集合。
class MainWindowObjectGraph
{
public:
	struct Node
	{
		QString id;
		QVector<QString> parentIds;
		QString primaryParentId;
		std::shared_ptr<BackendDataBase> data;
		QVector<QString> childIds;
	};

	static MainWindowObjectGraph build(
		const std::vector<std::shared_ptr<BackendDataBase>>& objects,
		const std::vector<std::pair<QString, QString>>& edges);

	const Node* node(const QString& id) const;
	const QVector<QString>& nodeOrder() const { return m_nodeOrder; }
	QVector<QString> subtreeIds(const QString& rootId) const;

private:
	QHash<QString, Node> m_nodes;
	QVector<QString> m_nodeOrder;
};
