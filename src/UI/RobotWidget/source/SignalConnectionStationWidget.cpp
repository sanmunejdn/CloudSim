/// @file SignalConnectionStationWidget.cpp
/// @brief SignalConnectionStationWidget 实现

#include "SignalConnectionStationWidget.h"

#include "IoSignalNetworkService.h"
#include "NamedSignalTable.h"

#include <QLabel>
#include <QKeyEvent>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <QtMath>

namespace
{
constexpr double kNodeW = 200.0;
constexpr double kHeaderH = 28.0;
constexpr double kPortStep = 22.0;
constexpr double kPortR = 5.0;
} // namespace

SignalConnectionStationWidget::SignalConnectionStationWidget(QWidget* parent) : QWidget(parent)
{
	setMouseTracking(true);
	setFocusPolicy(Qt::StrongFocus);
	auto* root = new QVBoxLayout(this);
	root->setContentsMargins(0, 0, 0, 0);
	root->setSpacing(0);
	m_status = new QLabel(this);
	m_status->setMargin(6);
	root->addWidget(m_status);
	root->addStretch(1);
	setMinimumHeight(240);
	updateStatus(QString());
}

void SignalConnectionStationWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	updateStatus(QString());
	update();
}

void SignalConnectionStationWidget::setNetwork(IoSignalNetworkService* network)
{
	if (m_network)
	{
		disconnect(m_network, nullptr, this, nullptr);
	}
	m_network = network;
	if (m_network)
	{
		connect(m_network, &IoSignalNetworkService::networkChanged, this,
				&SignalConnectionStationWidget::refreshFromNetwork);
	}
	refreshFromNetwork();
}

void SignalConnectionStationWidget::refreshFromNetwork()
{
	rebuildNodes();
	update();
}

void SignalConnectionStationWidget::updateStatus(const QString& text)
{
	if (!m_status)
	{
		return;
	}
	if (!text.isEmpty())
	{
		m_status->setText(text);
		return;
	}
	m_status->setText(m_useChinese ? QStringLiteral("连接站：从 DO（右）拖到 DI（左）。点选边后 Delete 删除。")
								   : QStringLiteral("Station: drag DO (right) to DI (left). Select edge + Delete."));
}

void SignalConnectionStationWidget::rebuildNodes()
{
	m_nodes.clear();
	if (!m_network)
	{
		return;
	}
	for (const QString& id : m_network->ownerIds())
	{
		NodeVisual n;
		n.ownerId = id;
		const RobotIo::NamedSignalTable* table = m_network->table(id);
		if (table)
		{
			for (const RobotIo::SignalDef& s : table->entries())
			{
				if (s.name.empty())
				{
					continue;
				}
				if (s.kind == RobotIo::SignalKind::DI)
				{
					n.diNames << QString::fromStdString(s.name);
				}
				else if (s.kind == RobotIo::SignalKind::DO)
				{
					n.doNames << QString::fromStdString(s.name);
				}
			}
		}
		const int rows = qMax(1, qMax(n.diNames.size(), n.doNames.size()));
		const double h = kHeaderH + 12.0 + rows * kPortStep;
		n.rect = QRectF(m_network->canvasX(id), m_network->canvasY(id), kNodeW, h);
		m_nodes.push_back(n);
	}
}

QPointF SignalConnectionStationWidget::toScene(const QPointF& view) const
{
	return (view - m_pan) / m_zoom;
}

QPointF SignalConnectionStationWidget::toView(const QPointF& scene) const
{
	return scene * m_zoom + m_pan;
}

int SignalConnectionStationWidget::hitNode(const QPointF& scene) const
{
	for (int i = m_nodes.size() - 1; i >= 0; --i)
	{
		if (m_nodes[i].rect.contains(scene))
		{
			return i;
		}
	}
	return -1;
}

QPointF SignalConnectionStationWidget::portPos(const NodeVisual& n, const bool isDo, const int index) const
{
	const double y = n.rect.top() + kHeaderH + 10.0 + index * kPortStep;
	const double x = isDo ? n.rect.right() : n.rect.left();
	return QPointF(x, y);
}

bool SignalConnectionStationWidget::hitPort(const QPointF& scene, PortRef* out) const
{
	if (!out)
	{
		return false;
	}
	for (const NodeVisual& n : m_nodes)
	{
		for (int i = 0; i < n.diNames.size(); ++i)
		{
			if (QLineF(scene, portPos(n, false, i)).length() <= kPortR + 4.0)
			{
				out->ownerId = n.ownerId;
				out->signalName = n.diNames[i];
				out->isDo = false;
				return true;
			}
		}
		for (int i = 0; i < n.doNames.size(); ++i)
		{
			if (QLineF(scene, portPos(n, true, i)).length() <= kPortR + 4.0)
			{
				out->ownerId = n.ownerId;
				out->signalName = n.doNames[i];
				out->isDo = true;
				return true;
			}
		}
	}
	return false;
}

void SignalConnectionStationWidget::paintEvent(QPaintEvent*)
{
	QPainter p(this);
	p.fillRect(rect(), QColor(245, 247, 250));
	p.translate(m_pan);
	p.scale(m_zoom, m_zoom);

	if (m_network)
	{
		for (const IoSignalWire& w : m_network->wires())
		{
			const NodeVisual* fromN = nullptr;
			const NodeVisual* toN = nullptr;
			int fromIdx = -1;
			int toIdx = -1;
			for (const NodeVisual& n : m_nodes)
			{
				if (n.ownerId == w.fromOwnerId)
				{
					fromN = &n;
					fromIdx = n.doNames.indexOf(w.fromSignal);
				}
				if (n.ownerId == w.toOwnerId)
				{
					toN = &n;
					toIdx = n.diNames.indexOf(w.toSignal);
				}
			}
			if (!fromN || !toN || fromIdx < 0 || toIdx < 0)
			{
				continue;
			}
			const QPointF a = portPos(*fromN, true, fromIdx);
			const QPointF b = portPos(*toN, false, toIdx);
			QPen pen(w.id == m_selectedWireId ? QColor(220, 80, 60) : QColor(60, 110, 180), 2.0);
			p.setPen(pen);
			QPainterPath path;
			path.moveTo(a);
			const double dx = qMax(40.0, qAbs(b.x() - a.x()) * 0.4);
			path.cubicTo(a + QPointF(dx, 0), b - QPointF(dx, 0), b);
			p.drawPath(path);
		}
	}

	for (const NodeVisual& n : m_nodes)
	{
		p.setPen(QPen(QColor(70, 70, 70), 1.2));
		p.setBrush(QColor(255, 255, 255));
		p.drawRoundedRect(n.rect, 6, 6);
		p.fillRect(QRectF(n.rect.left(), n.rect.top(), n.rect.width(), kHeaderH), QColor(230, 236, 245));
		const QString title = m_network ? m_network->displayName(n.ownerId) : n.ownerId;
		p.drawText(QRectF(n.rect.left() + 8, n.rect.top(), n.rect.width() - 16, kHeaderH),
				   Qt::AlignVCenter | Qt::AlignLeft, title);
		for (int i = 0; i < n.diNames.size(); ++i)
		{
			const QPointF c = portPos(n, false, i);
			p.setBrush(QColor(80, 160, 90));
			p.drawEllipse(c, kPortR, kPortR);
			p.drawText(QPointF(n.rect.left() + 12, c.y() + 4), n.diNames[i]);
		}
		for (int i = 0; i < n.doNames.size(); ++i)
		{
			const QPointF c = portPos(n, true, i);
			p.setBrush(QColor(70, 120, 200));
			p.drawEllipse(c, kPortR, kPortR);
			p.drawText(QRectF(n.rect.left(), c.y() - 10, n.rect.width() - 12, 20), Qt::AlignVCenter | Qt::AlignRight,
					   n.doNames[i]);
		}
	}

	if (m_wiring)
	{
		p.setPen(QPen(QColor(120, 80, 200), 1.5, Qt::DashLine));
		const NodeVisual* fromN = nullptr;
		int fromIdx = -1;
		for (const NodeVisual& n : m_nodes)
		{
			if (n.ownerId == m_wireFrom.ownerId)
			{
				fromN = &n;
				fromIdx = n.doNames.indexOf(m_wireFrom.signalName);
				break;
			}
		}
		if (fromN && fromIdx >= 0)
		{
			p.drawLine(portPos(*fromN, true, fromIdx), m_wireTo);
		}
	}
}

void SignalConnectionStationWidget::mousePressEvent(QMouseEvent* event)
{
	const QPointF scene = toScene(event->pos());
	if (event->button() == Qt::MiddleButton ||
		(event->button() == Qt::LeftButton && (event->modifiers() & Qt::AltModifier)))
	{
		m_panning = true;
		m_panLast = event->pos();
		return;
	}
	if (event->button() == Qt::LeftButton)
	{
		PortRef port;
		if (hitPort(scene, &port) && port.isDo)
		{
			m_wiring = true;
			m_wireFrom = port;
			m_wireTo = scene;
			m_selectedWireId.clear();
			update();
			return;
		}
		if (m_network)
		{
			m_selectedWireId.clear();
			for (const IoSignalWire& w : m_network->wires())
			{
				const NodeVisual* fromN = nullptr;
				const NodeVisual* toN = nullptr;
				int fromIdx = -1;
				int toIdx = -1;
				for (const NodeVisual& n : m_nodes)
				{
					if (n.ownerId == w.fromOwnerId)
					{
						fromN = &n;
						fromIdx = n.doNames.indexOf(w.fromSignal);
					}
					if (n.ownerId == w.toOwnerId)
					{
						toN = &n;
						toIdx = n.diNames.indexOf(w.toSignal);
					}
				}
				if (!fromN || !toN || fromIdx < 0 || toIdx < 0)
				{
					continue;
				}
				const QPointF a = portPos(*fromN, true, fromIdx);
				const QPointF b = portPos(*toN, false, toIdx);
				const QPointF mid = (a + b) * 0.5;
				if (QLineF(scene, mid).length() < 14.0)
				{
					m_selectedWireId = w.id;
					update();
					return;
				}
			}
		}
		m_dragNode = hitNode(scene);
		if (m_dragNode >= 0)
		{
			m_dragOffset = scene - m_nodes[m_dragNode].rect.topLeft();
		}
	}
}

void SignalConnectionStationWidget::mouseMoveEvent(QMouseEvent* event)
{
	const QPointF scene = toScene(event->pos());
	if (m_panning)
	{
		m_pan += event->pos() - m_panLast;
		m_panLast = event->pos();
		update();
		return;
	}
	if (m_wiring)
	{
		m_wireTo = scene;
		update();
		return;
	}
	if (m_dragNode >= 0 && m_dragNode < m_nodes.size())
	{
		const QPointF tl = scene - m_dragOffset;
		m_nodes[m_dragNode].rect.moveTopLeft(tl);
		if (m_network)
		{
			m_network->setCanvasPos(m_nodes[m_dragNode].ownerId, tl.x(), tl.y());
		}
		update();
	}
}

void SignalConnectionStationWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if (event->button() == Qt::MiddleButton || m_panning)
	{
		m_panning = false;
	}
	if (m_wiring && event->button() == Qt::LeftButton)
	{
		PortRef to;
		const QPointF scene = toScene(event->pos());
		if (hitPort(scene, &to) && !to.isDo && m_network)
		{
			IoSignalWire w;
			w.fromOwnerId = m_wireFrom.ownerId;
			w.fromSignal = m_wireFrom.signalName;
			w.toOwnerId = to.ownerId;
			w.toSignal = to.signalName;
			QString err;
			if (!m_network->addWire(w, &err))
			{
				updateStatus(err);
			}
			else
			{
				updateStatus(QString());
			}
		}
		m_wiring = false;
		update();
	}
	m_dragNode = -1;
}

void SignalConnectionStationWidget::wheelEvent(QWheelEvent* event)
{
	const double factor = event->angleDelta().y() > 0 ? 1.1 : (1.0 / 1.1);
	m_zoom = qBound(0.4, m_zoom * factor, 2.5);
	update();
}

void SignalConnectionStationWidget::keyPressEvent(QKeyEvent* event)
{
	if ((event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace) && m_network &&
		!m_selectedWireId.isEmpty())
	{
		m_network->removeWire(m_selectedWireId);
		m_selectedWireId.clear();
		updateStatus(QString());
		return;
	}
	QWidget::keyPressEvent(event);
}
