#ifndef WIDGET_SCENEHIERARCHYCANVASWIDGET_H
#define WIDGET_SCENEHIERARCHYCANVASWIDGET_H

/// @file SceneHierarchyCanvasWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 场景层级只读画布：Link 风格块 + 父子连线；可折叠；双击看属性

#include "widget_global.h"

#include "IRenderView.h"

#include <QHash>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QWidget>

class QMouseEvent;
class QPaintEvent;
class QResizeEvent;
class QWheelEvent;

class WIDGET_EXPORT SceneHierarchyCanvasWidget : public QWidget
{
	Q_OBJECT

public:
	explicit SceneHierarchyCanvasWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void clearGraph();
	void setEmptyHint(const QString& text);
	void setSceneGraph(const cloudsim::core::IRenderView::SceneNodeInfo& root);

signals:
	/// 双击带后端根的节点时发出，供 MainWindow 联动选中
	void backendNodeActivated(const QString& backendId);

protected:
	void paintEvent(QPaintEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseDoubleClickEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	void resizeEvent(QResizeEvent* event) override;

private:
	struct Node
	{
		QString id;
		QString title;
		QString subtitle;
		QRectF rect;
		bool isRoot = false;
		bool hasBackend = false;
		bool expanded = false; // 默认折叠
		bool visibleInLayout = false;
		QVector<int> childIndices;
		cloudsim::core::IRenderView::SceneNodeInfo detail;
	};
	struct Edge
	{
		QString from;
		QString to;
	};

	QPointF toScene(const QPointF& view) const;
	QPointF toView(const QPointF& scene) const;
	QPointF portPos(const Node& n, bool right) const;
	QRectF expandButtonRect(const Node& n) const;
	int hitNode(const QPointF& scene) const;
	int hitExpandButton(const QPointF& scene) const;
	void fitContentInView();
	void relayout();
	double layoutSubtree(int nodeIndex, int depth, double& yCursor);
	void rebuildFromRoot(const cloudsim::core::IRenderView::SceneNodeInfo& root);
	void showNodeDetailDialog(const Node& node);
	void toggleExpanded(int nodeIndex);

	bool m_useChinese = true;
	QString m_emptyHint;
	bool m_truncated = false;
	QVector<Node> m_nodes;
	QVector<Edge> m_edges;
	QHash<QString, int> m_idToIndex;
	QString m_selectedId;
	double m_zoom = 1.0;
	QPointF m_pan;
	int m_dragNode = -1;
	QPointF m_dragOffset;
	bool m_panning = false;
	QPointF m_panLast;
};

#endif
