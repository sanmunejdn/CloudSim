#ifndef ROBOTWIDGET_CUSTOMDEVICEASSEMBLYCANVASWIDGET_H
#define ROBOTWIDGET_CUSTOMDEVICEASSEMBLYCANVASWIDGET_H

/// @file CustomDeviceAssemblyCanvasWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 自定义设备组装画布：Link 块 + 运动副连线

#include "robotwidget_global.h"

#include "CustomDeviceBackendData.h"

#include <QHash>
#include <QPointF>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QWidget>

class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

class ROBOTWIDGET_EXPORT CustomDeviceAssemblyCanvasWidget : public QWidget
{
	Q_OBJECT

public:
	explicit CustomDeviceAssemblyCanvasWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setConnectionMode(bool on);
	bool connectionMode() const { return m_connectionMode; }

	QString addLinkBlock(const QString& displayName, const QString& geometryBackendId, const QPointF& pos,
						 bool fixed = false);
	bool removeSelected();
	void clearAll();
	void setSelectedFixed(bool fixed);

	QVector<CustomDeviceLink> links() const;
	QVector<CustomDeviceJoint> joints() const;
	void setGraph(const QVector<CustomDeviceLink>& links, const QVector<CustomDeviceJoint>& joints);

	QString selectedLinkId() const { return m_selectedLinkId; }
	QString selectedJointId() const { return m_selectedJointId; }
	bool updateSelectedJointMotion(const CustomDeviceAxisConfig& motion);
	bool setLinkRestInDeviceW0(const QString& linkId, const double rest[16]);

signals:
	void selectionChanged();
	void graphChanged();

protected:
	void paintEvent(QPaintEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;

private:
	struct Node
	{
		QString id;
		QString title;
		QString geometryId;
		bool fixed = false;
		QRectF rect;
		double restInDeviceW0[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	};
	struct Edge
	{
		QString id;
		QString from;
		QString to;
		CustomDeviceAxisConfig motion;
		double rest[16]{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};
	};

	QPointF toScene(const QPointF& view) const;
	QPointF toView(const QPointF& scene) const;
	int hitNode(const QPointF& scene) const;
	int hitEdge(const QPointF& scene) const;
	QPointF portPos(const Node& n, bool right) const;
	void ensureUniqueFixed(const QString& preferId);

	bool m_useChinese = true;
	bool m_connectionMode = false;
	double m_zoom = 1.0;
	QPointF m_pan;
	QVector<Node> m_nodes;
	QVector<Edge> m_edges;
	QString m_selectedLinkId;
	QString m_selectedJointId;
	int m_dragNode = -1;
	QPointF m_dragOffset;
	bool m_panning = false;
	QPointF m_panLast;
	bool m_wiring = false;
	QString m_wireFrom;
	QPointF m_wireTo;
	int m_idSeq = 1;
};

#endif // ROBOTWIDGET_CUSTOMDEVICEASSEMBLYCANVASWIDGET_H
