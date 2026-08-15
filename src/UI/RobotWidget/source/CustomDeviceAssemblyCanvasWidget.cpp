/// @file CustomDeviceAssemblyCanvasWidget.cpp
/// @brief 自定义设备组装画布实现

#include "CustomDeviceAssemblyCanvasWidget.h"

#include <QLineF>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QWheelEvent>

#include <cmath>
#include <cstring>

namespace
{
constexpr double kNodeW = 160.0;
constexpr double kNodeH = 72.0;

void identity16(double m[16])
{
	std::memset(m, 0, sizeof(double) * 16);
	m[0] = m[5] = m[10] = m[15] = 1.0;
}
} // namespace

CustomDeviceAssemblyCanvasWidget::CustomDeviceAssemblyCanvasWidget(QWidget* parent) : QWidget(parent)
{
	setMouseTracking(true);
	setFocusPolicy(Qt::StrongFocus);
	setMinimumSize(420, 320);
	setBackgroundRole(QPalette::Base);
	setAutoFillBackground(true);
}

void CustomDeviceAssemblyCanvasWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	update();
}

void CustomDeviceAssemblyCanvasWidget::setConnectionMode(const bool on)
{
	m_connectionMode = on;
	m_wiring = false;
	update();
}

QPointF CustomDeviceAssemblyCanvasWidget::toScene(const QPointF& view) const
{
	return (view - m_pan) / m_zoom;
}

QPointF CustomDeviceAssemblyCanvasWidget::toView(const QPointF& scene) const
{
	return scene * m_zoom + m_pan;
}

QPointF CustomDeviceAssemblyCanvasWidget::portPos(const Node& n, const bool right) const
{
	return QPointF(right ? n.rect.right() : n.rect.left(), n.rect.center().y());
}

int CustomDeviceAssemblyCanvasWidget::hitNode(const QPointF& scene) const
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

int CustomDeviceAssemblyCanvasWidget::hitEdge(const QPointF& scene) const
{
	for (int i = 0; i < m_edges.size(); ++i)
	{
		const Edge& e = m_edges[i];
		int fi = -1;
		int ti = -1;
		for (int n = 0; n < m_nodes.size(); ++n)
		{
			if (m_nodes[n].id == e.from)
			{
				fi = n;
			}
			if (m_nodes[n].id == e.to)
			{
				ti = n;
			}
		}
		if (fi < 0 || ti < 0)
		{
			continue;
		}
		const QPointF a = portPos(m_nodes[fi], true);
		const QPointF b = portPos(m_nodes[ti], false);
		const QPointF ab = b - a;
		const double ab2 = ab.x() * ab.x() + ab.y() * ab.y();
		double t = 0.0;
		if (ab2 > 1e-12)
		{
			const QPointF as = scene - a;
			t = (as.x() * ab.x() + as.y() * ab.y()) / ab2;
		}
		t = qBound(0.0, t, 1.0);
		const QPointF nearest = a + t * ab;
		if (QLineF(scene, nearest).length() < 10.0 / m_zoom)
		{
			return i;
		}
	}
	return -1;
}

void CustomDeviceAssemblyCanvasWidget::ensureUniqueFixed(const QString& preferId)
{
	for (Node& n : m_nodes)
	{
		n.fixed = false;
	}
	if (!preferId.isEmpty())
	{
		for (Node& n : m_nodes)
		{
			if (n.id == preferId)
			{
				n.fixed = true;
				return;
			}
		}
	}
	if (!m_nodes.isEmpty())
	{
		m_nodes[0].fixed = true;
	}
}

QString CustomDeviceAssemblyCanvasWidget::addLinkBlock(const QString& displayName, const QString& geometryBackendId,
													  const QPointF& pos, const bool fixed)
{
	Node n;
	n.id = QStringLiteral("L%1").arg(m_idSeq++);
	n.title = displayName.isEmpty() ? n.id : displayName;
	n.geometryId = geometryBackendId;
	n.fixed = fixed;
	n.rect = QRectF(pos.x() - kNodeW * 0.5, pos.y() - kNodeH * 0.5, kNodeW, kNodeH);
	if (fixed || m_nodes.isEmpty())
	{
		ensureUniqueFixed(n.id);
	}
	m_nodes.push_back(n);
	m_selectedLinkId = n.id;
	m_selectedJointId.clear();
	emit selectionChanged();
	emit graphChanged();
	update();
	return n.id;
}

bool CustomDeviceAssemblyCanvasWidget::removeSelected()
{
	if (!m_selectedJointId.isEmpty())
	{
		for (int i = 0; i < m_edges.size(); ++i)
		{
			if (m_edges[i].id == m_selectedJointId)
			{
				m_edges.removeAt(i);
				m_selectedJointId.clear();
				emit selectionChanged();
				emit graphChanged();
				update();
				return true;
			}
		}
	}
	if (!m_selectedLinkId.isEmpty())
	{
		const QString id = m_selectedLinkId;
		for (int i = m_edges.size() - 1; i >= 0; --i)
		{
			if (m_edges[i].from == id || m_edges[i].to == id)
			{
				m_edges.removeAt(i);
			}
		}
		for (int i = 0; i < m_nodes.size(); ++i)
		{
			if (m_nodes[i].id == id)
			{
				m_nodes.removeAt(i);
				break;
			}
		}
		m_selectedLinkId.clear();
		ensureUniqueFixed(QString());
		emit selectionChanged();
		emit graphChanged();
		update();
		return true;
	}
	return false;
}

void CustomDeviceAssemblyCanvasWidget::clearAll()
{
	m_nodes.clear();
	m_edges.clear();
	m_selectedLinkId.clear();
	m_selectedJointId.clear();
	emit selectionChanged();
	emit graphChanged();
	update();
}

void CustomDeviceAssemblyCanvasWidget::setSelectedFixed(const bool fixed)
{
	if (m_selectedLinkId.isEmpty())
	{
		return;
	}
	if (fixed)
	{
		ensureUniqueFixed(m_selectedLinkId);
	}
	else
	{
		for (Node& n : m_nodes)
		{
			if (n.id == m_selectedLinkId)
			{
				n.fixed = false;
			}
		}
		ensureUniqueFixed(QString());
	}
	emit graphChanged();
	update();
}

QVector<CustomDeviceLink> CustomDeviceAssemblyCanvasWidget::links() const
{
	QVector<CustomDeviceLink> out;
	out.reserve(m_nodes.size());
	for (const Node& n : m_nodes)
	{
		CustomDeviceLink L;
		L.id = n.id.toStdString();
		L.displayName = n.title.toStdString();
		L.geometryBackendId = n.geometryId.toStdString();
		L.fixed = n.fixed;
		L.canvasX = n.rect.center().x();
		L.canvasY = n.rect.center().y();
		identity16(L.restInDeviceW0);
		out.push_back(L);
	}
	return out;
}

QVector<CustomDeviceJoint> CustomDeviceAssemblyCanvasWidget::joints() const
{
	QVector<CustomDeviceJoint> out;
	out.reserve(m_edges.size());
	for (const Edge& e : m_edges)
	{
		CustomDeviceJoint J;
		J.id = e.id.toStdString();
		J.parentLinkId = e.from.toStdString();
		J.childLinkId = e.to.toStdString();
		J.motion = e.motion;
		std::memcpy(J.parentToChildRest, e.rest, sizeof(double) * 16);
		out.push_back(J);
	}
	return out;
}

void CustomDeviceAssemblyCanvasWidget::setGraph(const QVector<CustomDeviceLink>& links,
											   const QVector<CustomDeviceJoint>& joints)
{
	m_nodes.clear();
	m_edges.clear();
	int maxId = 0;
	for (const CustomDeviceLink& L : links)
	{
		Node n;
		n.id = QString::fromStdString(L.id);
		n.title = QString::fromStdString(L.displayName);
		n.geometryId = QString::fromStdString(L.geometryBackendId);
		n.fixed = L.fixed;
		n.rect = QRectF(L.canvasX - kNodeW * 0.5, L.canvasY - kNodeH * 0.5, kNodeW, kNodeH);
		m_nodes.push_back(n);
		if (n.id.startsWith(QLatin1Char('L')))
		{
			maxId = std::max(maxId, n.id.mid(1).toInt());
		}
	}
	for (const CustomDeviceJoint& J : joints)
	{
		Edge e;
		e.id = QString::fromStdString(J.id);
		e.from = QString::fromStdString(J.parentLinkId);
		e.to = QString::fromStdString(J.childLinkId);
		e.motion = J.motion;
		std::memcpy(e.rest, J.parentToChildRest, sizeof(double) * 16);
		m_edges.push_back(e);
		if (e.id.startsWith(QLatin1Char('J')))
		{
			maxId = std::max(maxId, e.id.mid(1).toInt());
		}
	}
	m_idSeq = maxId + 1;
	update();
}

bool CustomDeviceAssemblyCanvasWidget::updateSelectedJointMotion(const CustomDeviceAxisConfig& motion)
{
	if (m_selectedJointId.isEmpty())
	{
		return false;
	}
	for (Edge& e : m_edges)
	{
		if (e.id == m_selectedJointId)
		{
			e.motion = motion;
			normalizeCustomDeviceAxisConfig(e.motion);
			emit graphChanged();
			update();
			return true;
		}
	}
	return false;
}

void CustomDeviceAssemblyCanvasWidget::paintEvent(QPaintEvent*)
{
	QPainter p(this);
	p.fillRect(rect(), QColor(QStringLiteral("#EEF1F5")));
	p.setRenderHint(QPainter::Antialiasing, true);

	p.save();
	p.translate(m_pan);
	p.scale(m_zoom, m_zoom);

	for (const Edge& e : m_edges)
	{
		int fi = -1;
		int ti = -1;
		for (int i = 0; i < m_nodes.size(); ++i)
		{
			if (m_nodes[i].id == e.from)
			{
				fi = i;
			}
			if (m_nodes[i].id == e.to)
			{
				ti = i;
			}
		}
		if (fi < 0 || ti < 0)
		{
			continue;
		}
		const QPointF a = portPos(m_nodes[fi], true);
		const QPointF b = portPos(m_nodes[ti], false);
		const bool sel = e.id == m_selectedJointId;
		const bool rotate = e.motion.motionType == CustomDeviceMotionType::Rotate;
		QColor col = rotate ? QColor(QStringLiteral("#C7771A")) : QColor(QStringLiteral("#0066CC"));
		if (sel)
		{
			col = QColor(QStringLiteral("#C62828"));
		}
		// 仅描边；若 brush 残留（上一根线的标签白底），drawPath 会把曲线围成的区域填白
		p.setBrush(Qt::NoBrush);
		p.setPen(QPen(col, (sel ? 2.6 : 2.0) / m_zoom));
		QPainterPath path;
		path.moveTo(a);
		const double dx = (b.x() - a.x()) * 0.45;
		path.cubicTo(a + QPointF(dx, 0), b - QPointF(dx, 0), b);
		p.drawPath(path);
		const QString label =
			rotate ? (m_useChinese ? QStringLiteral("旋转副 %1").arg(e.id) : QStringLiteral("Revolute %1").arg(e.id))
				   : (m_useChinese ? QStringLiteral("移动副 %1").arg(e.id) : QStringLiteral("Prismatic %1").arg(e.id));
		const QPointF mid = path.pointAtPercent(0.5);
		QRectF badge(0, 0, 100, 24);
		badge.moveCenter(mid);
		p.setBrush(sel ? QColor(QStringLiteral("#FFEBEE")) : QColor(255, 255, 255, 242));
		p.setPen(QPen(col, 1.2 / m_zoom));
		p.drawRoundedRect(badge, 10, 10);
		p.setPen(QColor(QStringLiteral("#1C1C1E")));
		QFont f = font();
		f.setPointSizeF(8.5);
		f.setBold(true);
		p.setFont(f);
		p.drawText(badge, Qt::AlignCenter, label);
	}

	if (m_wiring)
	{
		int fi = -1;
		for (int i = 0; i < m_nodes.size(); ++i)
		{
			if (m_nodes[i].id == m_wireFrom)
			{
				fi = i;
			}
		}
		if (fi >= 0)
		{
			p.setPen(QPen(QColor(QStringLiteral("#0066CC")), 1.6 / m_zoom, Qt::DashLine));
			p.drawLine(portPos(m_nodes[fi], true), m_wireTo);
		}
	}

	for (const Node& n : m_nodes)
	{
		const bool sel = n.id == m_selectedLinkId;
		// 轻阴影压住节点层次
		p.setPen(Qt::NoPen);
		p.setBrush(QColor(28, 28, 30, 28));
		p.drawRoundedRect(n.rect.translated(0, 1.5 / m_zoom), 10, 10);
		p.setPen(QPen(sel ? QColor(QStringLiteral("#0066CC")) : QColor(QStringLiteral("#C5CDD6")),
					  (sel ? 2.4 : 1.2) / m_zoom));
		p.setBrush(sel ? QColor(QStringLiteral("#F3F8FF")) : QColor(QStringLiteral("#FFFFFF")));
		p.drawRoundedRect(n.rect, 10, 10);
		p.setPen(QColor(QStringLiteral("#1C1C1E")));
		QFont f = font();
		f.setBold(true);
		p.setFont(f);
		p.drawText(n.rect.adjusted(10, 8, -10, -28), Qt::AlignLeft | Qt::AlignTop, n.title);
		f.setBold(false);
		f.setPointSizeF(8);
		p.setFont(f);
		p.setPen(QColor(QStringLiteral("#6B7280")));
		p.drawText(n.rect.adjusted(10, 30, -10, -8), Qt::AlignLeft | Qt::AlignTop, n.geometryId);
		if (n.fixed)
		{
			QRectF badge(n.rect.right() - 48, n.rect.top() + 6, 40, 16);
			p.setBrush(QColor(QStringLiteral("#1F9D63")));
			p.setPen(Qt::NoPen);
			p.drawRoundedRect(badge, 6, 6);
			p.setPen(Qt::white);
			p.drawText(badge, Qt::AlignCenter, m_useChinese ? QStringLiteral("固定") : QStringLiteral("Fixed"));
		}
		p.setBrush(sel ? QColor(QStringLiteral("#0066CC")) : QColor(QStringLiteral("#4B5563")));
		p.setPen(QPen(QColor(QStringLiteral("#FFFFFF")), 1.2 / m_zoom));
		p.drawEllipse(portPos(n, false), 5, 5);
		p.drawEllipse(portPos(n, true), 5, 5);
	}
	p.restore();

	const QString tip = m_connectionMode
							? (m_useChinese ? QStringLiteral("连接模式：从父块拖到子块") : QStringLiteral("Connect: drag parent → child"))
							: (m_useChinese ? QStringLiteral("选择 / 拖动块") : QStringLiteral("Select / drag blocks"));
	const QRect tipBar(0, height() - 28, width(), 28);
	p.fillRect(tipBar, QColor(255, 255, 255, 210));
	p.setPen(QColor(QStringLiteral("#DADCE0")));
	p.drawLine(0, tipBar.top(), width(), tipBar.top());
	p.setPen(QColor(QStringLiteral("#4B5563")));
	p.drawText(tipBar.adjusted(12, 0, -12, 0), Qt::AlignLeft | Qt::AlignVCenter, tip);
}

void CustomDeviceAssemblyCanvasWidget::mousePressEvent(QMouseEvent* event)
{
	const QPointF scene = toScene(event->pos());
	if (event->button() == Qt::MiddleButton ||
		(event->button() == Qt::LeftButton && event->modifiers() & Qt::AltModifier))
	{
		m_panning = true;
		m_panLast = event->pos();
		return;
	}
	if (event->button() != Qt::LeftButton)
	{
		return;
	}

	if (m_connectionMode)
	{
		const int ni = hitNode(scene);
		if (ni >= 0)
		{
			m_wiring = true;
			m_wireFrom = m_nodes[ni].id;
			m_wireTo = scene;
			update();
		}
		return;
	}

	const int ei = hitEdge(scene);
	if (ei >= 0)
	{
		m_selectedJointId = m_edges[ei].id;
		m_selectedLinkId.clear();
		emit selectionChanged();
		update();
		return;
	}
	const int ni = hitNode(scene);
	if (ni >= 0)
	{
		m_selectedLinkId = m_nodes[ni].id;
		m_selectedJointId.clear();
		m_dragNode = ni;
		m_dragOffset = scene - m_nodes[ni].rect.topLeft();
		emit selectionChanged();
		update();
		return;
	}
	m_selectedLinkId.clear();
	m_selectedJointId.clear();
	emit selectionChanged();
	update();
}

void CustomDeviceAssemblyCanvasWidget::mouseMoveEvent(QMouseEvent* event)
{
	if (m_panning)
	{
		const QPointF d = event->pos() - m_panLast;
		m_pan += d;
		m_panLast = event->pos();
		update();
		return;
	}
	const QPointF scene = toScene(event->pos());
	if (m_wiring)
	{
		m_wireTo = scene;
		update();
		return;
	}
	if (m_dragNode >= 0 && m_dragNode < m_nodes.size())
	{
		m_nodes[m_dragNode].rect.moveTopLeft(scene - m_dragOffset);
		emit graphChanged();
		update();
	}
}

void CustomDeviceAssemblyCanvasWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if (event->button() == Qt::MiddleButton || m_panning)
	{
		m_panning = false;
		return;
	}
	if (m_wiring && event->button() == Qt::LeftButton)
	{
		const QPointF scene = toScene(event->pos());
		const int ti = hitNode(scene);
		if (ti >= 0 && m_nodes[ti].id != m_wireFrom)
		{
			bool exists = false;
			for (const Edge& e : m_edges)
			{
				if (e.to == m_nodes[ti].id)
				{
					exists = true;
					break;
				}
			}
			if (!exists)
			{
				Edge e;
				e.id = QStringLiteral("J%1").arg(m_idSeq++);
				e.from = m_wireFrom;
				e.to = m_nodes[ti].id;
				e.motion = makeDefaultCustomDeviceRotateAxis();
				e.motion.displayName = e.id.toStdString();
				e.motion.jointName = e.id.toStdString();
				identity16(e.rest);
				m_edges.push_back(e);
				m_selectedJointId = e.id;
				m_selectedLinkId.clear();
				emit selectionChanged();
				emit graphChanged();
			}
		}
		m_wiring = false;
		update();
		return;
	}
	m_dragNode = -1;
}

void CustomDeviceAssemblyCanvasWidget::wheelEvent(QWheelEvent* event)
{
	const double factor = event->angleDelta().y() > 0 ? 1.1 : (1.0 / 1.1);
	const QPointF before = toScene(event->position());
	m_zoom = qBound(0.35, m_zoom * factor, 2.8);
	const QPointF after = toScene(event->position());
	m_pan += (after - before) * m_zoom;
	update();
}
