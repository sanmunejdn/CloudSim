#ifndef ROBOTWIDGET_SIGNALCONNECTIONSTATIONWIDGET_H
#define ROBOTWIDGET_SIGNALCONNECTIONSTATIONWIDGET_H

/// @file SignalConnectionStationWidget.h
/// @note 自研代码仅供研究学习，不得商用；商用请联系 921857463@qq.com
/// @brief 信号连接站：Owner 节点 + DO→DI 拖线

#include "robotwidget_global.h"

#include <QPointF>
#include <QRectF>
#include <QString>
#include <QVector>
#include <QWidget>

class IoSignalNetworkService;
class QLabel;
class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

class ROBOTWIDGET_EXPORT SignalConnectionStationWidget : public QWidget
{
	Q_OBJECT

public:
	explicit SignalConnectionStationWidget(QWidget* parent = nullptr);

	void setUseChinese(bool chinese);
	void setNetwork(IoSignalNetworkService* network);
	void refreshFromNetwork();

protected:
	void paintEvent(QPaintEvent* event) override;
	void mousePressEvent(QMouseEvent* event) override;
	void mouseMoveEvent(QMouseEvent* event) override;
	void mouseReleaseEvent(QMouseEvent* event) override;
	void wheelEvent(QWheelEvent* event) override;
	void keyPressEvent(QKeyEvent* event) override;

private:
	struct PortRef
	{
		QString ownerId;
		QString signalName;
		bool isDo = false;
	};
	struct NodeVisual
	{
		QString ownerId;
		QRectF rect;
		QVector<QString> diNames;
		QVector<QString> doNames;
	};

	void rebuildNodes();
	QPointF toScene(const QPointF& view) const;
	QPointF toView(const QPointF& scene) const;
	int hitNode(const QPointF& scene) const;
	bool hitPort(const QPointF& scene, PortRef* out) const;
	QPointF portPos(const NodeVisual& n, bool isDo, int index) const;
	void updateStatus(const QString& text);

	bool m_useChinese = true;
	IoSignalNetworkService* m_network = nullptr;
	double m_zoom = 1.0;
	QPointF m_pan;
	QVector<NodeVisual> m_nodes;
	bool m_panning = false;
	QPointF m_panLast;
	int m_dragNode = -1;
	QPointF m_dragOffset;
	bool m_wiring = false;
	PortRef m_wireFrom;
	QPointF m_wireTo;
	QString m_selectedWireId;
	QLabel* m_status = nullptr;
};

#endif // ROBOTWIDGET_SIGNALCONNECTIONSTATIONWIDGET_H
