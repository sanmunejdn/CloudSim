/// @file ProcessFlowCanvasWidget.cpp
/// @brief 自研流程画布实现

#include "ProcessFlowCanvasWidget.h"

#include <QContextMenuEvent>
#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QFile>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QLineEdit>
#include <QLineF>
#include <QMenu>
#include <QMimeData>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPainterPathStroker>
#include <QWheelEvent>

#include <QHash>
#include <QQueue>
#include <QSet>

#include <algorithm>
#include <cmath>
#include <limits>

namespace
{
constexpr double kMinZoom = 0.25;
constexpr double kMaxZoom = 3.5;
constexpr double kNodeWidth = 168.0;
constexpr double kNodeHeight = 78.0;
constexpr double kLayoutHGap = 80.0;
constexpr double kLayoutVGap = 40.0;

QColor defaultNodeColor(int index)
{
	switch (index % 5)
	{
	case 1:
		return QColor(QStringLiteral("#1F9D63"));
	case 2:
		return QColor(QStringLiteral("#D87516"));
	case 3:
		return QColor(QStringLiteral("#7A5CFA"));
	case 4:
		return QColor(QStringLiteral("#E9573F"));
	default:
		return QColor(QStringLiteral("#2E7DD1"));
	}
}

QPointF eventPos(const QMouseEvent* event)
{
	return event->localPos();
}

QPointF eventPos(const QWheelEvent* event)
{
	return event->posF();
}
} // namespace

ProcessFlowCanvasWidget::ProcessFlowCanvasWidget(QWidget* parent) : QWidget(parent)
{
	setMouseTracking(true);
	setFocusPolicy(Qt::StrongFocus);
	setMinimumSize(400, 300);
	setAcceptDrops(true);
}

int ProcessFlowCanvasWidget::addNode(const QString& title, const QString& subtitle, const QColor& color,
									 const QPointF& position, const QString& kind)
{
	Node node;
	node.id = m_nextNodeId++;
	node.title = title;
	node.subtitle = subtitle;
	node.color = color.isValid() ? color : defaultNodeColor(node.id);
	node.rect = QRectF(position, QSizeF(kNodeWidth, kNodeHeight));
	node.props = ProcessFlowNodeProps::defaultsForKind(kind);
	m_nodes.push_back(node);
	setSelectedNode(node.id);
	emitGraphChanged();
	update();
	return node.id;
}

void ProcessFlowCanvasWidget::addEdge(int from, int to, const QString& label)
{
	if (from == to || !findNode(from) || !findNode(to))
	{
		return;
	}
	for (const Edge& edge : m_edges)
	{
		if (edge.from == from && edge.to == to)
		{
			return;
		}
	}
	m_edges.push_back({from, to, label});
	setSelectedEdge(m_edges.size() - 1);
	emitGraphChanged();
	update();
}

void ProcessFlowCanvasWidget::removeSelectedItem()
{
	if (m_selectedEdgeIndex >= 0)
	{
		removeSelectedEdge();
		return;
	}
	removeSelectedNode();
}

void ProcessFlowCanvasWidget::removeSelectedNode()
{
	if (m_selectedNodeId < 0)
	{
		return;
	}
	const int id = m_selectedNodeId;
	m_nodes.erase(std::remove_if(m_nodes.begin(), m_nodes.end(), [id](const Node& n) { return n.id == id; }),
				  m_nodes.end());
	m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
								 [id](const Edge& e) { return e.from == id || e.to == id; }),
				  m_edges.end());
	m_selectedEdgeIndex = -1;
	setSelectedNode(-1);
	emitGraphChanged();
	update();
}

void ProcessFlowCanvasWidget::removeSelectedEdge()
{
	if (m_selectedEdgeIndex < 0 || m_selectedEdgeIndex >= m_edges.size())
	{
		return;
	}
	m_edges.removeAt(m_selectedEdgeIndex);
	setSelectedEdge(-1);
	emitGraphChanged();
	update();
}

bool ProcessFlowCanvasWidget::removeNodeById(int id)
{
	if (id < 0 || !findNode(id))
		return false;
	m_nodes.erase(std::remove_if(m_nodes.begin(), m_nodes.end(), [id](const Node& n) { return n.id == id; }),
				  m_nodes.end());
	m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
								 [id](const Edge& e) { return e.from == id || e.to == id; }),
				  m_edges.end());
	if (m_selectedNodeId == id)
		setSelectedNode(-1);
	emitGraphChanged();
	update();
	return true;
}

bool ProcessFlowCanvasWidget::removeEdge(int from, int to)
{
	const int before = m_edges.size();
	m_edges.erase(std::remove_if(m_edges.begin(), m_edges.end(),
								 [from, to](const Edge& e) { return e.from == from && e.to == to; }),
				  m_edges.end());
	if (m_edges.size() == before)
		return false;
	setSelectedEdge(-1);
	emitGraphChanged();
	update();
	return true;
}

void ProcessFlowCanvasWidget::clearGraph()
{
	m_nodes.clear();
	m_edges.clear();
	m_selectedNodeId = -1;
	m_selectedEdgeIndex = -1;
	m_nextNodeId = 1;
	emitGraphChanged();
	emit nodeSelected(-1, QString());
	update();
}

void ProcessFlowCanvasWidget::autoLayout()
{
	if (m_nodes.isEmpty())
	{
		return;
	}

	QHash<int, int> idToIndex;
	idToIndex.reserve(m_nodes.size());
	for (int i = 0; i < m_nodes.size(); ++i)
	{
		idToIndex.insert(m_nodes[i].id, i);
	}

	QHash<int, QVector<int>> succ;
	QHash<int, QVector<int>> pred;
	QHash<int, int> indegree;
	for (const Node& n : m_nodes)
	{
		indegree.insert(n.id, 0);
		succ.insert(n.id, {});
		pred.insert(n.id, {});
	}
	for (const Edge& e : m_edges)
	{
		if (!idToIndex.contains(e.from) || !idToIndex.contains(e.to) || e.from == e.to)
		{
			continue;
		}
		succ[e.from].append(e.to);
		pred[e.to].append(e.from);
		indegree[e.to] = indegree.value(e.to) + 1;
	}

	if (m_edges.isEmpty())
	{
		const int cols = std::max(1, static_cast<int>(std::ceil(std::sqrt(static_cast<double>(m_nodes.size())))));
		for (int i = 0; i < m_nodes.size(); ++i)
		{
			const int col = i % cols;
			const int row = i / cols;
			m_nodes[i].rect.moveTopLeft(QPointF(col * (kNodeWidth + kLayoutHGap), row * (kNodeHeight + kLayoutVGap)));
		}
		fitToView();
		emitGraphChanged();
		update();
		return;
	}

	QVector<int> roots;
	for (const Node& n : m_nodes)
	{
		if (n.props.kind == QLatin1String("start"))
		{
			roots.append(n.id);
		}
	}
	if (roots.isEmpty())
	{
		for (const Node& n : m_nodes)
		{
			if (indegree.value(n.id) == 0)
			{
				roots.append(n.id);
			}
		}
	}
	if (roots.isEmpty())
	{
		int minId = m_nodes.first().id;
		for (const Node& n : m_nodes)
		{
			minId = std::min(minId, n.id);
		}
		roots.append(minId);
	}

	// 最长路径定层：Kahn 拓扑序上取前驱最大层+1；环内剩余节点再补
	QHash<int, int> workIndeg = indegree;
	QQueue<int> q;
	QSet<int> rootSet;
	for (int id : roots)
	{
		rootSet.insert(id);
		q.enqueue(id);
	}
	for (auto it = indegree.cbegin(); it != indegree.cend(); ++it)
	{
		if (it.value() == 0 && !rootSet.contains(it.key()))
		{
			q.enqueue(it.key());
		}
	}

	QHash<int, int> layer;
	QVector<int> topo;
	while (!q.isEmpty())
	{
		const int u = q.dequeue();
		if (layer.contains(u))
		{
			continue;
		}
		int best = 0;
		bool hasPred = false;
		for (int p : pred.value(u))
		{
			if (!layer.contains(p))
			{
				continue;
			}
			hasPred = true;
			best = std::max(best, layer.value(p) + 1);
		}
		layer.insert(u, hasPred ? best : 0);
		topo.append(u);
		for (int v : succ.value(u))
		{
			workIndeg[v] = workIndeg.value(v) - 1;
			if (workIndeg.value(v) <= 0 && !layer.contains(v))
			{
				q.enqueue(v);
			}
		}
	}
	for (const Node& n : m_nodes)
	{
		if (layer.contains(n.id))
		{
			continue;
		}
		int best = 0;
		bool hasPred = false;
		for (int p : pred.value(n.id))
		{
			if (!layer.contains(p))
			{
				continue;
			}
			hasPred = true;
			best = std::max(best, layer.value(p) + 1);
		}
		layer.insert(n.id, hasPred ? best : 0);
		topo.append(n.id);
	}

	int maxLayer = 0;
	for (auto it = layer.cbegin(); it != layer.cend(); ++it)
	{
		maxLayer = std::max(maxLayer, it.value());
	}
	QVector<QVector<int>> buckets(maxLayer + 1);
	for (int id : topo)
	{
		buckets[layer.value(id)].append(id);
	}
	for (QVector<int>& b : buckets)
	{
		std::sort(b.begin(), b.end());
	}

	auto barycenter = [](const QVector<int>& neighbors, const QHash<int, int>& pos) -> double
	{
		if (neighbors.isEmpty())
		{
			return std::numeric_limits<double>::quiet_NaN();
		}
		double sum = 0.0;
		int count = 0;
		for (int n : neighbors)
		{
			if (!pos.contains(n))
			{
				continue;
			}
			sum += pos.value(n);
			++count;
		}
		return count > 0 ? sum / count : std::numeric_limits<double>::quiet_NaN();
	};

	for (int pass = 0; pass < 2; ++pass)
	{
		for (int L = 1; L <= maxLayer; ++L)
		{
			QHash<int, int> prevPos;
			for (int i = 0; i < buckets[L - 1].size(); ++i)
			{
				prevPos.insert(buckets[L - 1][i], i);
			}
			QVector<QPair<double, int>> keyed;
			keyed.reserve(buckets[L].size());
			for (int id : buckets[L])
			{
				const double key = barycenter(pred.value(id), prevPos);
				keyed.append({std::isnan(key) ? id : key, id});
			}
			std::stable_sort(keyed.begin(), keyed.end(),
							 [](const QPair<double, int>& a, const QPair<double, int>& b) { return a.first < b.first; });
			buckets[L].clear();
			for (const auto& k : keyed)
			{
				buckets[L].append(k.second);
			}
		}
		for (int L = maxLayer - 1; L >= 0; --L)
		{
			QHash<int, int> nextPos;
			for (int i = 0; i < buckets[L + 1].size(); ++i)
			{
				nextPos.insert(buckets[L + 1][i], i);
			}
			QVector<QPair<double, int>> keyed;
			keyed.reserve(buckets[L].size());
			for (int id : buckets[L])
			{
				const double key = barycenter(succ.value(id), nextPos);
				keyed.append({std::isnan(key) ? id : key, id});
			}
			std::stable_sort(keyed.begin(), keyed.end(),
							 [](const QPair<double, int>& a, const QPair<double, int>& b) { return a.first < b.first; });
			buckets[L].clear();
			for (const auto& k : keyed)
			{
				buckets[L].append(k.second);
			}
		}
	}

	int maxRows = 1;
	for (const QVector<int>& b : buckets)
	{
		maxRows = std::max(maxRows, b.size());
	}
	const double totalH = maxRows * kNodeHeight + (maxRows - 1) * kLayoutVGap;
	for (int L = 0; L <= maxLayer; ++L)
	{
		const QVector<int>& b = buckets[L];
		const double colH = b.size() * kNodeHeight + std::max(0, b.size() - 1) * kLayoutVGap;
		const double y0 = (totalH - colH) * 0.5;
		const double x = L * (kNodeWidth + kLayoutHGap);
		for (int i = 0; i < b.size(); ++i)
		{
			const int idx = idToIndex.value(b[i], -1);
			if (idx < 0)
			{
				continue;
			}
			const double y = y0 + i * (kNodeHeight + kLayoutVGap);
			m_nodes[idx].rect.moveTopLeft(QPointF(x, y));
		}
	}

	fitToView();
	emitGraphChanged();
	update();
}

void ProcessFlowCanvasWidget::fitToView()
{
	const QRectF bounds = graphBounds();
	if (bounds.isNull() || width() <= 0 || height() <= 0)
	{
		return;
	}
	const double xScale = (width() - 96.0) / std::max(1.0, bounds.width());
	const double yScale = (height() - 96.0) / std::max(1.0, bounds.height());
	m_zoom = std::clamp(std::min(xScale, yScale), kMinZoom, kMaxZoom);
	m_panOffset = QPointF(width() / 2.0, height() / 2.0) - bounds.center() * m_zoom;
	m_needInitialFit = false;
	emit viewChanged(m_zoom);
	update();
}

void ProcessFlowCanvasWidget::zoomIn()
{
	m_zoom *= 1.18;
	clampZoom();
	emit viewChanged(m_zoom);
	update();
}

void ProcessFlowCanvasWidget::zoomOut()
{
	m_zoom /= 1.18;
	clampZoom();
	emit viewChanged(m_zoom);
	update();
}

void ProcessFlowCanvasWidget::resetView()
{
	m_zoom = 1.0;
	m_panOffset = QPointF(40.0, 40.0);
	emit viewChanged(m_zoom);
	update();
}

void ProcessFlowCanvasWidget::setGridVisible(bool visible)
{
	m_gridVisible = visible;
	update();
}

void ProcessFlowCanvasWidget::setConnectionMode(bool enabled)
{
	m_connectionMode = enabled;
	m_connecting = false;
	m_connectionFromId = -1;
	update();
}

bool ProcessFlowCanvasWidget::exportJson(const QString& fileName) const
{
	QFile file(fileName);
	if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate))
	{
		return false;
	}
	file.write(QJsonDocument(toJson()).toJson(QJsonDocument::Indented));
	return true;
}

QJsonObject ProcessFlowCanvasWidget::toJson() const
{
	QJsonArray nodes;
	for (const Node& node : m_nodes)
	{
		QJsonObject item;
		item.insert(QStringLiteral("id"), node.id);
		item.insert(QStringLiteral("title"), node.title);
		item.insert(QStringLiteral("subtitle"), node.subtitle);
		item.insert(QStringLiteral("color"), node.color.name(QColor::HexRgb).toUpper());
		item.insert(QStringLiteral("x"), node.rect.x());
		item.insert(QStringLiteral("y"), node.rect.y());
		item.insert(QStringLiteral("width"), node.rect.width());
		item.insert(QStringLiteral("height"), node.rect.height());
		item.insert(QStringLiteral("props"), node.props.toJson());
		nodes.append(item);
	}
	QJsonArray edges;
	for (const Edge& edge : m_edges)
	{
		QJsonObject item;
		item.insert(QStringLiteral("from"), edge.from);
		item.insert(QStringLiteral("to"), edge.to);
		item.insert(QStringLiteral("label"), edge.label);
		edges.append(item);
	}
	QJsonObject root;
	root.insert(QStringLiteral("version"), 1);
	root.insert(QStringLiteral("nodes"), nodes);
	root.insert(QStringLiteral("edges"), edges);
	if (!m_jobSetJson.isEmpty())
	{
		root.insert(QStringLiteral("jobSet"), m_jobSetJson);
	}
	return root;
}

bool ProcessFlowCanvasWidget::fromJson(const QJsonObject& root)
{
	m_nodes.clear();
	m_edges.clear();
	m_selectedNodeId = -1;
	m_selectedEdgeIndex = -1;
	m_nextNodeId = 1;
	m_jobSetJson = root.value(QStringLiteral("jobSet")).toObject();
	clearPlayback();

	const QJsonArray nodes = root.value(QStringLiteral("nodes")).toArray();
	for (const QJsonValue& v : nodes)
	{
		const QJsonObject item = v.toObject();
		Node node;
		node.id = item.value(QStringLiteral("id")).toInt(m_nextNodeId);
		node.title = item.value(QStringLiteral("title")).toString();
		node.subtitle = item.value(QStringLiteral("subtitle")).toString();
		node.color = QColor(item.value(QStringLiteral("color")).toString(QStringLiteral("#2E7DD1")));
		const double w = item.value(QStringLiteral("width")).toDouble(kNodeWidth);
		const double h = item.value(QStringLiteral("height")).toDouble(kNodeHeight);
		node.rect = QRectF(item.value(QStringLiteral("x")).toDouble(), item.value(QStringLiteral("y")).toDouble(), w, h);
		if (item.contains(QStringLiteral("props")) && item.value(QStringLiteral("props")).isObject())
		{
			node.props = ProcessFlowNodeProps::fromJson(item.value(QStringLiteral("props")).toObject());
		}
		else
		{
			node.props = ProcessFlowNodeProps::defaultsForKind(
				ProcessFlowNodeProps::inferKindFromTitle(node.title, node.subtitle));
		}
		m_nodes.push_back(node);
		m_nextNodeId = std::max(m_nextNodeId, node.id + 1);
	}

	const QJsonArray edges = root.value(QStringLiteral("edges")).toArray();
	for (const QJsonValue& v : edges)
	{
		const QJsonObject item = v.toObject();
		Edge edge;
		edge.from = item.value(QStringLiteral("from")).toInt();
		edge.to = item.value(QStringLiteral("to")).toInt();
		edge.label = item.value(QStringLiteral("label")).toString();
		if (edge.from != edge.to && findNode(edge.from) && findNode(edge.to))
		{
			m_edges.push_back(edge);
		}
	}

	m_needInitialFit = true;
	emitGraphChanged();
	emit nodeSelected(-1, QString());
	update();
	return true;
}

bool ProcessFlowCanvasWidget::nodeProps(int id, ProcessFlowNodeProps* out) const
{
	const Node* node = findNode(id);
	if (!node || !out)
	{
		return false;
	}
	*out = node->props;
	return true;
}

bool ProcessFlowCanvasWidget::setNodeProps(int id, const ProcessFlowNodeProps& props)
{
	Node* node = findNode(id);
	if (!node)
	{
		return false;
	}
	node->props = props;
	emitGraphChanged();
	update();
	return true;
}

void ProcessFlowCanvasWidget::paintEvent(QPaintEvent*)
{
	QPainter painter(this);
	painter.setRenderHint(QPainter::Antialiasing, true);
	painter.fillRect(rect(), QColor(QStringLiteral("#F5F7FA")));
	if (m_gridVisible)
	{
		drawGrid(&painter);
	}
	if (m_needInitialFit && !m_nodes.isEmpty())
	{
		fitToView();
	}
	for (int i = 0; i < m_edges.size(); ++i)
	{
		drawEdge(&painter, m_edges.at(i), i == m_selectedEdgeIndex);
	}
	drawConnectionDraft(&painter);
	for (const Node& node : m_nodes)
	{
		drawNode(&painter, node, node.id == m_selectedNodeId);
	}
	for (auto it = m_tokenPositions.begin(); it != m_tokenPositions.end(); ++it)
	{
		const QPointF w = sceneToWidget(it.value());
		painter.setPen(Qt::NoPen);
		painter.setBrush(QColor(QStringLiteral("#2563EB")));
		painter.drawEllipse(w, 5.0 * std::max(0.25, m_zoom), 5.0 * std::max(0.25, m_zoom));
	}
}

void ProcessFlowCanvasWidget::resizeEvent(QResizeEvent*)
{
	if (m_needInitialFit && !m_nodes.isEmpty())
	{
		fitToView();
	}
}

void ProcessFlowCanvasWidget::contextMenuEvent(QContextMenuEvent* event)
{
	const QPointF scenePoint = widgetToScene(event->pos());
	const int nodeId = hitNode(scenePoint);
	QMenu menu(this);
	if (nodeId >= 0)
	{
		setSelectedNode(nodeId);
		QAction* renameAction = menu.addAction(QStringLiteral("重命名节点"));
		QAction* connectAction = menu.addAction(QStringLiteral("从此节点开始连线"));
		QAction* deleteAction = menu.addAction(QStringLiteral("删除节点"));
		const QAction* chosen = menu.exec(event->globalPos());
		if (chosen == renameAction)
		{
			editNodeTitle(nodeId);
		}
		else if (chosen == connectAction)
		{
			m_connectionMode = true;
			m_connecting = true;
			m_connectionFromId = nodeId;
			m_connectionEnd = scenePoint;
			update();
		}
		else if (chosen == deleteAction)
		{
			removeSelectedNode();
		}
		return;
	}
	const int edgeIndex = hitEdge(scenePoint);
	if (edgeIndex >= 0)
	{
		setSelectedEdge(edgeIndex);
		QAction* editAction = menu.addAction(QStringLiteral("编辑连线标签"));
		QAction* deleteAction = menu.addAction(QStringLiteral("删除连线"));
		const QAction* chosen = menu.exec(event->globalPos());
		if (chosen == editAction)
		{
			editEdgeLabel(edgeIndex);
		}
		else if (chosen == deleteAction)
		{
			removeSelectedEdge();
		}
		return;
	}
	QAction* addAction = menu.addAction(QStringLiteral("在此处添加节点"));
	QAction* layoutAction = menu.addAction(QStringLiteral("自动排版"));
	QAction* fitAction = menu.addAction(QStringLiteral("适应窗口"));
	const QAction* chosen = menu.exec(event->globalPos());
	if (chosen == addAction)
	{
		addNode(QStringLiteral("节点 %1").arg(m_nextNodeId), QStringLiteral("右键添加"),
				defaultNodeColor(m_nextNodeId), scenePoint - QPointF(kNodeWidth / 2.0, kNodeHeight / 2.0),
				QStringLiteral("station"));
	}
	else if (chosen == layoutAction)
	{
		autoLayout();
	}
	else if (chosen == fitAction)
	{
		fitToView();
	}
}

void ProcessFlowCanvasWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
	const QPointF scenePoint = widgetToScene(eventPos(event));
	const int nodeId = hitNode(scenePoint);
	if (nodeId >= 0)
	{
		setSelectedNode(nodeId);
		editNodeTitle(nodeId);
		return;
	}
	const int edgeIndex = hitEdge(scenePoint);
	if (edgeIndex >= 0)
	{
		setSelectedEdge(edgeIndex);
		editEdgeLabel(edgeIndex);
		return;
	}
	QWidget::mouseDoubleClickEvent(event);
}

void ProcessFlowCanvasWidget::mousePressEvent(QMouseEvent* event)
{
	setFocus();
	m_lastWidgetPos = eventPos(event);
	m_lastScenePos = widgetToScene(m_lastWidgetPos);
	if (event->button() == Qt::MiddleButton || event->modifiers().testFlag(Qt::AltModifier))
	{
		m_panning = true;
		setCursor(Qt::ClosedHandCursor);
		return;
	}
	if (event->button() != Qt::LeftButton)
	{
		return;
	}
	const int id = hitNode(m_lastScenePos);
	if (m_connectionMode)
	{
		if (id < 0)
		{
			m_connecting = false;
			m_connectionFromId = -1;
			update();
			return;
		}
		if (!m_connecting)
		{
			setSelectedNode(id);
			m_connecting = true;
			m_connectionFromId = id;
			m_connectionEnd = m_lastScenePos;
			update();
			return;
		}
		if (id != m_connectionFromId)
		{
			addEdge(m_connectionFromId, id, QStringLiteral("next"));
			setSelectedNode(id);
		}
		m_connecting = false;
		m_connectionFromId = -1;
		update();
		return;
	}
	if (id >= 0)
	{
		setSelectedNode(id);
		m_draggingNode = true;
		if (event->modifiers().testFlag(Qt::ShiftModifier))
		{
			m_connecting = true;
			m_connectionFromId = id;
			m_connectionEnd = m_lastScenePos;
			m_draggingNode = false;
		}
		return;
	}
	setSelectedEdge(hitEdge(m_lastScenePos));
	m_draggingNode = false;
}

void ProcessFlowCanvasWidget::mouseMoveEvent(QMouseEvent* event)
{
	const QPointF widgetPos = eventPos(event);
	const QPointF scenePos = widgetToScene(widgetPos);
	if (m_panning)
	{
		m_panOffset += widgetPos - m_lastWidgetPos;
		m_lastWidgetPos = widgetPos;
		update();
		return;
	}
	if (m_connecting)
	{
		m_connectionEnd = scenePos;
		update();
		return;
	}
	if (m_draggingNode && m_selectedNodeId >= 0)
	{
		if (Node* node = findNode(m_selectedNodeId))
		{
			node->rect.translate(scenePos - m_lastScenePos);
			emit nodeSelected(node->id, node->title);
		}
		m_lastScenePos = scenePos;
		update();
	}
}

void ProcessFlowCanvasWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if (event->button() == Qt::MiddleButton || m_panning)
	{
		m_panning = false;
		unsetCursor();
	}
	if (event->button() == Qt::LeftButton && m_connecting && !m_connectionMode)
	{
		const int target = hitNode(widgetToScene(eventPos(event)));
		if (target >= 0 && target != m_connectionFromId)
		{
			addEdge(m_connectionFromId, target, QStringLiteral("next"));
		}
		m_connecting = false;
		m_connectionFromId = -1;
		update();
	}
	m_draggingNode = false;
}

void ProcessFlowCanvasWidget::wheelEvent(QWheelEvent* event)
{
	const QPointF widgetPoint = eventPos(event);
	const QPointF scenePoint = widgetToScene(widgetPoint);
	m_zoom *= (event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15);
	clampZoom();
	m_panOffset = widgetPoint - scenePoint * m_zoom;
	emit viewChanged(m_zoom);
	update();
}

void ProcessFlowCanvasWidget::keyPressEvent(QKeyEvent* event)
{
	if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace)
	{
		removeSelectedItem();
		return;
	}
	if (event->key() == Qt::Key_Escape)
	{
		m_connecting = false;
		m_connectionFromId = -1;
		update();
		return;
	}
	QWidget::keyPressEvent(event);
}

QSize ProcessFlowCanvasWidget::minimumSizeHint() const
{
	return QSize(400, 300);
}

QRectF ProcessFlowCanvasWidget::nodeRect(const Node& node) const
{
	return node.rect.normalized();
}

QPointF ProcessFlowCanvasWidget::sceneToWidget(const QPointF& point) const
{
	return point * m_zoom + m_panOffset;
}

QPointF ProcessFlowCanvasWidget::widgetToScene(const QPointF& point) const
{
	return (point - m_panOffset) / std::max(0.001, m_zoom);
}

QRectF ProcessFlowCanvasWidget::sceneToWidget(const QRectF& rect) const
{
	return QRectF(sceneToWidget(rect.topLeft()), sceneToWidget(rect.bottomRight())).normalized();
}

int ProcessFlowCanvasWidget::hitNode(const QPointF& scenePoint) const
{
	for (int i = m_nodes.size() - 1; i >= 0; --i)
	{
		if (nodeRect(m_nodes.at(i)).contains(scenePoint))
		{
			return m_nodes.at(i).id;
		}
	}
	return -1;
}

int ProcessFlowCanvasWidget::hitEdge(const QPointF& scenePoint) const
{
	QPainterPathStroker stroker;
	stroker.setWidth(std::max(8.0, 12.0 / std::max(0.25, m_zoom)));
	stroker.setCapStyle(Qt::RoundCap);
	stroker.setJoinStyle(Qt::RoundJoin);
	for (int i = m_edges.size() - 1; i >= 0; --i)
	{
		if (stroker.createStroke(edgePath(m_edges.at(i))).contains(scenePoint))
		{
			return i;
		}
	}
	return -1;
}

ProcessFlowCanvasWidget::Node* ProcessFlowCanvasWidget::findNode(int id)
{
	for (Node& node : m_nodes)
	{
		if (node.id == id)
		{
			return &node;
		}
	}
	return nullptr;
}

const ProcessFlowCanvasWidget::Node* ProcessFlowCanvasWidget::findNode(int id) const
{
	for (const Node& node : m_nodes)
	{
		if (node.id == id)
		{
			return &node;
		}
	}
	return nullptr;
}

QRectF ProcessFlowCanvasWidget::graphBounds() const
{
	if (m_nodes.isEmpty())
	{
		return {};
	}
	QRectF bounds = nodeRect(m_nodes.first());
	for (int i = 1; i < m_nodes.size(); ++i)
	{
		bounds = bounds.united(nodeRect(m_nodes.at(i)));
	}
	return bounds.adjusted(-80, -60, 80, 60);
}

QPointF ProcessFlowCanvasWidget::nodeAnchor(const Node& node, bool output) const
{
	const QRectF rect = nodeRect(node);
	return output ? QPointF(rect.right(), rect.center().y()) : QPointF(rect.left(), rect.center().y());
}

QPainterPath ProcessFlowCanvasWidget::edgePath(const Edge& edge) const
{
	const Node* from = findNode(edge.from);
	const Node* to = findNode(edge.to);
	if (!from || !to)
	{
		return {};
	}
	const QPointF start = nodeAnchor(*from, true);
	const QPointF end = nodeAnchor(*to, false);
	const double handle = std::max(80.0, std::abs(end.x() - start.x()) * 0.45);
	QPainterPath path(start);
	path.cubicTo(QPointF(start.x() + handle, start.y()), QPointF(end.x() - handle, end.y()), end);
	return path;
}

void ProcessFlowCanvasWidget::setSelectedNode(int id)
{
	m_selectedNodeId = id;
	m_selectedEdgeIndex = -1;
	if (const Node* node = findNode(id))
	{
		emit nodeSelected(node->id, node->title);
	}
	else
	{
		emit nodeSelected(-1, QString());
	}
	update();
}

void ProcessFlowCanvasWidget::setSelectedEdge(int index)
{
	m_selectedEdgeIndex = index;
	m_selectedNodeId = -1;
	emit nodeSelected(-1, QString());
	update();
}

void ProcessFlowCanvasWidget::editNodeTitle(int id)
{
	Node* node = findNode(id);
	if (!node)
	{
		return;
	}
	bool ok = false;
	const QString title =
		QInputDialog::getText(this, QStringLiteral("重命名节点"), QStringLiteral("节点名称"), QLineEdit::Normal,
							  node->title, &ok);
	if (!ok || title.trimmed().isEmpty())
	{
		return;
	}
	node->title = title.trimmed();
	emit nodeSelected(node->id, node->title);
	update();
}

void ProcessFlowCanvasWidget::editEdgeLabel(int index)
{
	if (index < 0 || index >= m_edges.size())
	{
		return;
	}
	Edge& edge = m_edges[index];
	bool ok = false;
	const QString label =
		QInputDialog::getText(this, QStringLiteral("编辑连线标签"), QStringLiteral("连线标签"), QLineEdit::Normal,
							  edge.label, &ok);
	if (!ok)
	{
		return;
	}
	edge.label = label.trimmed();
	update();
}

void ProcessFlowCanvasWidget::emitGraphChanged()
{
	emit graphChanged(nodeCount(), edgeCount());
}

void ProcessFlowCanvasWidget::clampZoom()
{
	m_zoom = std::clamp(m_zoom, kMinZoom, kMaxZoom);
}

void ProcessFlowCanvasWidget::drawGrid(QPainter* painter) const
{
	painter->save();
	painter->setPen(QPen(QColor(QStringLiteral("#E5EBF0")), 1));
	const double step = 32.0 * m_zoom;
	if (step < 8.0)
	{
		painter->restore();
		return;
	}
	const double startX = std::fmod(m_panOffset.x(), step);
	const double startY = std::fmod(m_panOffset.y(), step);
	for (double x = startX; x < width(); x += step)
	{
		painter->drawLine(QPointF(x, 0), QPointF(x, height()));
	}
	for (double y = startY; y < height(); y += step)
	{
		painter->drawLine(QPointF(0, y), QPointF(width(), y));
	}
	painter->restore();
}

void ProcessFlowCanvasWidget::drawEdge(QPainter* painter, const Edge& edge, bool selected) const
{
	const Node* from = findNode(edge.from);
	const Node* to = findNode(edge.to);
	if (!from || !to)
	{
		return;
	}
	QPainterPath path;
	const QPainterPath scenePath = edgePath(edge);
	for (int i = 0; i < scenePath.elementCount(); ++i)
	{
		const QPainterPath::Element element = scenePath.elementAt(i);
		if (element.isMoveTo())
		{
			path.moveTo(sceneToWidget(QPointF(element.x, element.y)));
		}
		else if (element.isCurveTo())
		{
			const QPainterPath::Element c1 = scenePath.elementAt(i);
			const QPainterPath::Element c2 = scenePath.elementAt(i + 1);
			const QPainterPath::Element endElement = scenePath.elementAt(i + 2);
			path.cubicTo(sceneToWidget(QPointF(c1.x, c1.y)), sceneToWidget(QPointF(c2.x, c2.y)),
						 sceneToWidget(QPointF(endElement.x, endElement.y)));
			i += 2;
		}
	}
	const QPointF end = sceneToWidget(nodeAnchor(*to, false));
	painter->save();
	if (selected)
	{
		painter->setPen(QPen(QColor(QStringLiteral("#111827")), 6));
		painter->drawPath(path);
	}
	painter->setPen(QPen(QColor(QStringLiteral("#8EA0AD")), selected ? 4 : 2));
	painter->setBrush(Qt::NoBrush);
	painter->drawPath(path);
	painter->drawLine(QLineF(QPointF(end.x() - 16, end.y() - 7), end));
	painter->drawLine(QLineF(QPointF(end.x() - 16, end.y() + 7), end));
	if (!edge.label.isEmpty())
	{
		const QPointF center = path.pointAtPercent(0.5);
		const QRectF labelRect(center.x() - 36, center.y() - 12, 72, 24);
		painter->setPen(Qt::NoPen);
		painter->setBrush(QColor(QStringLiteral("#FFFFFF")));
		painter->drawRoundedRect(labelRect, 4, 4);
		painter->setPen(QColor(QStringLiteral("#66727C")));
		painter->drawText(labelRect, Qt::AlignCenter, edge.label);
	}
	painter->restore();
}

void ProcessFlowCanvasWidget::drawNode(QPainter* painter, const Node& node, bool selected) const
{
	const QRectF rect = sceneToWidget(nodeRect(node));
	// 边距随 zoom 缩放；固定像素 + AlignVCenter 会在放大后把三行挤到同一垂直中线
	const double s = std::max(0.25, m_zoom);
	painter->save();
	painter->setPen(QPen(selected ? QColor(QStringLiteral("#111827"))
								  : (m_busyNodeIds.contains(node.id) ? QColor(QStringLiteral("#DC2626"))
																	: node.color.darker(115)),
						 selected ? 3 : (m_busyNodeIds.contains(node.id) ? 3 : 2)));
	painter->setBrush(m_busyNodeIds.contains(node.id) ? QColor(QStringLiteral("#FEF2F2"))
													  : QColor(QStringLiteral("#FFFFFF")));
	const double cornerR = 8.0 * s;
	painter->drawRoundedRect(rect, cornerR, cornerR);
	// 色条圆角须与外框一致；独立 drawRoundedRect(4*s) 会在侧边顶出半圆
	const double barW = 9.0 * s;
	QPainterPath cardClip;
	cardClip.addRoundedRect(rect, cornerR, cornerR);
	painter->save();
	painter->setClipPath(cardClip);
	painter->setPen(Qt::NoPen);
	painter->setBrush(node.color);
	painter->drawRect(QRectF(rect.left(), rect.top(), barW, rect.height()));
	painter->restore();

	const double textLeft = rect.left() + barW + 8.0 * s;
	const double textWidth = std::max(8.0, rect.right() - 12.0 * s - textLeft);
	double y = rect.top() + 8.0 * s;

	painter->setPen(QColor(QStringLiteral("#1F2933")));
	QFont titleFont = painter->font();
	titleFont.setPointSizeF(std::max(8.0, 11.0 * s));
	titleFont.setBold(true);
	painter->setFont(titleFont);
	const QFontMetricsF titleFm(painter->font());
	painter->drawText(QRectF(textLeft, y, textWidth, titleFm.height()), Qt::AlignLeft | Qt::AlignVCenter, node.title);
	y += titleFm.height() + 2.0 * s;

	painter->setPen(QColor(QStringLiteral("#66727C")));
	QFont subtitleFont = painter->font();
	subtitleFont.setPointSizeF(std::max(7.0, 9.0 * s));
	subtitleFont.setBold(false);
	painter->setFont(subtitleFont);
	const QFontMetricsF subtitleFm(painter->font());
	painter->drawText(QRectF(textLeft, y, textWidth, subtitleFm.height()), Qt::AlignLeft | Qt::AlignVCenter,
					  node.subtitle);
	y += subtitleFm.height() + 2.0 * s;

	QString metrics;
	if (node.props.kind == QStringLiteral("conveyor"))
	{
		metrics = QStringLiteral("运=%1s").arg(node.props.cycleTimeSec, 0, 'f', 0);
	}
	else
	{
		metrics = QStringLiteral("T=%1s").arg(node.props.cycleTimeSec, 0, 'f', 0);
	}
	if (ProcessFlowNodeProps::isBufferKind(node.props.kind) || node.props.inventoryQty > 0.0)
	{
		metrics += QStringLiteral("  库存=%1").arg(node.props.inventoryQty, 0, 'f', 0);
	}
	painter->setPen(QColor(QStringLiteral("#8B9AA8")));
	QFont metricFont = painter->font();
	metricFont.setPointSizeF(std::max(6.0, 8.0 * s));
	painter->setFont(metricFont);
	const QFontMetricsF metricFm(painter->font());
	painter->drawText(QRectF(textLeft, y, textWidth, metricFm.height()), Qt::AlignLeft | Qt::AlignVCenter, metrics);

	const double portR = 4.0 * s;
	painter->setPen(QPen(QColor(QStringLiteral("#CDD7DF")), 1));
	painter->setBrush(QColor(QStringLiteral("#FFFFFF")));
	painter->drawEllipse(sceneToWidget(nodeAnchor(node, false)), portR, portR);
	painter->drawEllipse(sceneToWidget(nodeAnchor(node, true)), portR, portR);
	painter->restore();
}

void ProcessFlowCanvasWidget::drawConnectionDraft(QPainter* painter) const
{
	if (!m_connecting)
	{
		return;
	}
	const Node* from = findNode(m_connectionFromId);
	if (!from)
	{
		return;
	}
	const QPointF start = sceneToWidget(nodeAnchor(*from, true));
	const QPointF end = sceneToWidget(m_connectionEnd);
	const double handle = std::max(80.0, std::abs(end.x() - start.x()) * 0.45);
	QPainterPath path(start);
	path.cubicTo(QPointF(start.x() + handle, start.y()), QPointF(end.x() - handle, end.y()), end);
	painter->save();
	painter->setPen(QPen(QColor(QStringLiteral("#2E7DD1")), 2, Qt::DashLine));
	painter->drawPath(path);
	painter->restore();
}

void ProcessFlowCanvasWidget::dragEnterEvent(QDragEnterEvent* event)
{
	if (event->mimeData() && event->mimeData()->hasFormat(QString::fromLatin1(processFlowNodeMimeType())))
	{
		event->acceptProposedAction();
	}
}

void ProcessFlowCanvasWidget::dragMoveEvent(QDragMoveEvent* event)
{
	if (event->mimeData() && event->mimeData()->hasFormat(QString::fromLatin1(processFlowNodeMimeType())))
	{
		event->acceptProposedAction();
	}
}

void ProcessFlowCanvasWidget::dropEvent(QDropEvent* event)
{
	if (!event->mimeData() || !event->mimeData()->hasFormat(QString::fromLatin1(processFlowNodeMimeType())))
	{
		return;
	}
	const QByteArray raw = event->mimeData()->data(QString::fromLatin1(processFlowNodeMimeType()));
	const QJsonObject o = QJsonDocument::fromJson(raw).object();
	const QString kind = o.value(QStringLiteral("kind")).toString(QStringLiteral("station"));
	const QString title = o.value(QStringLiteral("title")).toString(ProcessFlowNodeProps::displayNameZh(kind));
	const QString subtitle = o.value(QStringLiteral("subtitle")).toString();
	const QColor color(o.value(QStringLiteral("color")).toString());
	const QPointF scenePos = widgetToScene(event->pos());
	const QPointF topLeft(scenePos.x() - kNodeWidth * 0.5, scenePos.y() - kNodeHeight * 0.5);
	const int id = addNode(title, subtitle, color, topLeft, kind);
	setSelectedNode(id);
	event->acceptProposedAction();
}

void ProcessFlowCanvasWidget::setJobSetJson(const QJsonObject& jobSet)
{
	m_jobSetJson = jobSet;
	emitGraphChanged();
}

void ProcessFlowCanvasWidget::clearPlayback()
{
	m_playbackStats = SimStatistics();
	m_playbackTime = 0.0;
	m_playbackTimeMax = 1.0;
	m_busyNodeIds.clear();
	m_tokenPositions.clear();
	update();
}

void ProcessFlowCanvasWidget::setPlaybackTrace(const SimStatistics& stats)
{
	m_playbackStats = stats;
	m_playbackTimeMax = std::max(1.0, std::max(stats.makespan, stats.horizonSec));
	for (const OperationTraceItem& it : stats.trace.items)
	{
		m_playbackTimeMax = std::max(m_playbackTimeMax, it.end);
	}
	setPlaybackTime(0.0);
}

void ProcessFlowCanvasWidget::setPlaybackTime(double t)
{
	m_playbackTime = std::clamp(t, 0.0, m_playbackTimeMax);
	m_busyNodeIds.clear();
	m_tokenPositions.clear();
	for (const OperationTraceItem& it : m_playbackStats.trace.items)
	{
		if (it.start <= m_playbackTime && m_playbackTime < it.end)
		{
			m_busyNodeIds.insert(it.machineNodeId);
			if (const Node* n = findNode(it.machineNodeId))
			{
				m_tokenPositions.insert(it.jobId, n->rect.center());
			}
		}
	}
	update();
}
