#ifndef PROCESSFLOWPLUGIN_PROCESSFLOWCANVASWIDGET_H
#define PROCESSFLOWPLUGIN_PROCESSFLOWCANVASWIDGET_H

/// @file ProcessFlowCanvasWidget.h
/// @brief 自研流程画布（参考 NodeFlowDemo 交互，无外部库依赖）

#include "ProcessFlowNodeProps.h"
#include "sim/SimStatistics.h"

#include <QColor>
#include <QHash>
#include <QJsonObject>
#include <QPainterPath>
#include <QPointF>
#include <QRectF>
#include <QSet>
#include <QString>
#include <QVector>
#include <QWidget>

class QContextMenuEvent;
class QDragEnterEvent;
class QDragMoveEvent;
class QDropEvent;
class QKeyEvent;
class QMouseEvent;
class QPaintEvent;
class QPainter;
class QResizeEvent;
class QWheelEvent;

class ProcessFlowCanvasWidget final : public QWidget
{
	Q_OBJECT

public:
	struct Node
	{
		int id = 0;
		QString title;
		QString subtitle;
		QColor color;
		QRectF rect;
		ProcessFlowNodeProps props;
	};

	struct Edge
	{
		int from = 0;
		int to = 0;
		QString label;
	};

	explicit ProcessFlowCanvasWidget(QWidget* parent = nullptr);

	int addNode(const QString& title, const QString& subtitle, const QColor& color, const QPointF& position,
				const QString& kind = QStringLiteral("station"));
	void addEdge(int from, int to, const QString& label = QString());
	void removeSelectedItem();
	void removeSelectedNode();
	void removeSelectedEdge();
	void clearGraph();
	void fitToView();
	void autoLayout();
	void zoomIn();
	void zoomOut();
	void resetView();
	void setGridVisible(bool visible);
	void setConnectionMode(bool enabled);
	bool exportJson(const QString& fileName) const;
	QJsonObject toJson() const;
	bool fromJson(const QJsonObject& root);

	bool nodeProps(int id, ProcessFlowNodeProps* out) const;
	bool setNodeProps(int id, const ProcessFlowNodeProps& props);

	void setJobSetJson(const QJsonObject& jobSet);
	QJsonObject jobSetJson() const { return m_jobSetJson; }

	void setPlaybackTrace(const SimStatistics& stats);
	void clearPlayback();
	void setPlaybackTime(double t);
	double playbackTime() const { return m_playbackTime; }
	double playbackTimeMax() const { return m_playbackTimeMax; }

	int selectedNodeId() const { return m_selectedNodeId; }
	int nodeCount() const { return m_nodes.size(); }
	int edgeCount() const { return m_edges.size(); }

signals:
	void graphChanged(int nodeCount, int edgeCount);
	void nodeSelected(int id, const QString& title);
	void viewChanged(double zoom);

protected:
	void paintEvent(QPaintEvent* event) override;
	void resizeEvent(QResizeEvent* event) override;
	void contextMenuEvent(QContextMenuEvent* event) override;
	void mouseDoubleClickEvent(QMouseEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	void keyPressEvent(QKeyEvent* event) override;
	void dragEnterEvent(QDragEnterEvent* event) override;
	void dragMoveEvent(QDragMoveEvent* event) override;
	void dropEvent(QDropEvent* event) override;
	QSize minimumSizeHint() const override;

private:
	QRectF nodeRect(const Node& node) const;
	QPointF sceneToWidget(const QPointF& point) const;
	QPointF widgetToScene(const QPointF& point) const;
	QRectF sceneToWidget(const QRectF& rect) const;
	int hitNode(const QPointF& scenePoint) const;
	int hitEdge(const QPointF& scenePoint) const;
	Node* findNode(int id);
	const Node* findNode(int id) const;
	QRectF graphBounds() const;
	QPointF nodeAnchor(const Node& node, bool output) const;
	QPainterPath edgePath(const Edge& edge) const;
	void setSelectedNode(int id);
	void setSelectedEdge(int index);
	void editNodeTitle(int id);
	void editEdgeLabel(int index);
	void emitGraphChanged();
	void clampZoom();
	void drawGrid(QPainter* painter) const;
	void drawEdge(QPainter* painter, const Edge& edge, bool selected) const;
	void drawNode(QPainter* painter, const Node& node, bool selected) const;
	void drawConnectionDraft(QPainter* painter) const;

	QVector<Node> m_nodes;
	QVector<Edge> m_edges;
	double m_zoom = 1.0;
	QPointF m_panOffset;
	bool m_gridVisible = true;
	bool m_draggingNode = false;
	bool m_panning = false;
	bool m_connecting = false;
	bool m_connectionMode = false;
	bool m_needInitialFit = true;
	QPointF m_lastWidgetPos;
	QPointF m_lastScenePos;
	QPointF m_connectionEnd;
	int m_selectedNodeId = -1;
	int m_selectedEdgeIndex = -1;
	int m_connectionFromId = -1;
	int m_nextNodeId = 1;
	QJsonObject m_jobSetJson;
	SimStatistics m_playbackStats;
	double m_playbackTime = 0.0;
	double m_playbackTimeMax = 1.0;
	QSet<int> m_busyNodeIds;
	QHash<int, QPointF> m_tokenPositions; // jobId -> scene pos
};

#endif // PROCESSFLOWPLUGIN_PROCESSFLOWCANVASWIDGET_H
