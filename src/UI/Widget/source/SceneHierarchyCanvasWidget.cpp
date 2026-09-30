/// @file SceneHierarchyCanvasWidget.cpp
/// @brief 场景层级只读画布实现

#include "SceneHierarchyCanvasWidget.h"

#include "OsgSceneNodeI18n.h"

#include <QDialog>
#include <QDialogButtonBox>
#include <QFormLayout>
#include <QFrame>
#include <QLabel>
#include <QLineF>
#include <QMouseEvent>
#include <QPainter>
#include <QPainterPath>
#include <QPlainTextEdit>
#include <QResizeEvent>
#include <QScrollArea>
#include <QVBoxLayout>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <functional>

namespace
{
constexpr double kNodeW = 160.0;
constexpr double kNodeH = 72.0;
constexpr double kHGap = 72.0;
constexpr double kVGap = 20.0;
constexpr int kMaxNodes = 200;
constexpr double kExpandBtn = 18.0;

QPlainTextEdit* makeMonoBlock(QWidget* parent, const QString& text)
{
	auto* edit = new QPlainTextEdit(parent);
	edit->setReadOnly(true);
	edit->setPlainText(text.isEmpty() ? QStringLiteral("—") : text);
	edit->setMaximumHeight(110);
	QFont f = edit->font();
	f.setFamily(QStringLiteral("Consolas"));
	f.setStyleHint(QFont::Monospace);
	f.setPointSizeF(8.5);
	edit->setFont(f);
	return edit;
}
} // namespace

SceneHierarchyCanvasWidget::SceneHierarchyCanvasWidget(QWidget* parent) : QWidget(parent)
{
	setMouseTracking(true);
	setFocusPolicy(Qt::StrongFocus);
	setMinimumSize(160, 160);
	setBackgroundRole(QPalette::Base);
	setAutoFillBackground(true);
}

void SceneHierarchyCanvasWidget::setUseChinese(const bool chinese)
{
	m_useChinese = chinese;
	if (!m_nodes.isEmpty())
	{
		for (Node& n : m_nodes)
		{
			const QString rawTitle =
				n.detail.displayName.isEmpty()
					? (n.detail.name.isEmpty() ? n.detail.className : n.detail.name)
					: n.detail.displayName;
			n.title = OsgSceneNodeI18n::translateNodeName(rawTitle, m_useChinese);
			const QString typeSrc =
				n.detail.hasBackend && !n.detail.backendClassName.isEmpty() ? n.detail.backendClassName
																			: n.detail.className;
			n.subtitle = OsgSceneNodeI18n::translateClassName(typeSrc, m_useChinese);
		}
	}
	update();
}

void SceneHierarchyCanvasWidget::clearGraph()
{
	m_nodes.clear();
	m_edges.clear();
	m_idToIndex.clear();
	m_selectedId.clear();
	m_truncated = false;
	m_emptyHint.clear();
	update();
}

void SceneHierarchyCanvasWidget::setEmptyHint(const QString& text)
{
	m_nodes.clear();
	m_edges.clear();
	m_idToIndex.clear();
	m_selectedId.clear();
	m_truncated = false;
	m_emptyHint = text;
	update();
}

QPointF SceneHierarchyCanvasWidget::toScene(const QPointF& view) const
{
	return (view - m_pan) / m_zoom;
}

QPointF SceneHierarchyCanvasWidget::toView(const QPointF& scene) const
{
	return scene * m_zoom + m_pan;
}

QPointF SceneHierarchyCanvasWidget::portPos(const Node& n, const bool right) const
{
	return QPointF(right ? n.rect.right() : n.rect.left(), n.rect.center().y());
}

QRectF SceneHierarchyCanvasWidget::expandButtonRect(const Node& n) const
{
	// 右侧端口旁：有子节点才可点
	return QRectF(n.rect.right() - kExpandBtn - 6.0, n.rect.center().y() - kExpandBtn * 0.5, kExpandBtn, kExpandBtn);
}

int SceneHierarchyCanvasWidget::hitNode(const QPointF& scene) const
{
	for (int i = m_nodes.size() - 1; i >= 0; --i)
	{
		if (!m_nodes[i].visibleInLayout)
		{
			continue;
		}
		if (m_nodes[i].rect.contains(scene))
		{
			return i;
		}
	}
	return -1;
}

int SceneHierarchyCanvasWidget::hitExpandButton(const QPointF& scene) const
{
	for (int i = m_nodes.size() - 1; i >= 0; --i)
	{
		const Node& n = m_nodes[i];
		if (!n.visibleInLayout || n.childIndices.isEmpty())
		{
			continue;
		}
		if (expandButtonRect(n).contains(scene))
		{
			return i;
		}
	}
	return -1;
}

void SceneHierarchyCanvasWidget::toggleExpanded(const int nodeIndex)
{
	if (nodeIndex < 0 || nodeIndex >= m_nodes.size())
	{
		return;
	}
	Node& n = m_nodes[nodeIndex];
	if (n.childIndices.isEmpty())
	{
		return;
	}
	n.expanded = !n.expanded;
	relayout();
	update();
}

void SceneHierarchyCanvasWidget::rebuildFromRoot(const cloudsim::core::IRenderView::SceneNodeInfo& root)
{
	m_nodes.clear();
	m_edges.clear();
	m_idToIndex.clear();
	m_selectedId.clear();
	m_truncated = false;
	m_emptyHint.clear();

	std::function<int(const cloudsim::core::IRenderView::SceneNodeInfo&, bool)> addNode;
	addNode = [&](const cloudsim::core::IRenderView::SceneNodeInfo& info, const bool isRoot) -> int
	{
		if (m_nodes.size() >= kMaxNodes)
		{
			m_truncated = true;
			return -1;
		}
		Node n;
		n.id = QStringLiteral("N%1").arg(m_nodes.size());
		n.detail = info;
		n.detail.children.clear();
		n.hasBackend = info.hasBackend;
		n.isRoot = isRoot;
		n.expanded = false;
		const QString rawTitle =
			info.displayName.isEmpty() ? (info.name.isEmpty() ? info.className : info.name) : info.displayName;
		n.title = OsgSceneNodeI18n::translateNodeName(rawTitle, m_useChinese);
		if (n.title.isEmpty())
		{
			n.title = n.id;
		}
		const QString typeSrc =
			info.hasBackend && !info.backendClassName.isEmpty() ? info.backendClassName : info.className;
		n.subtitle = OsgSceneNodeI18n::translateClassName(typeSrc, m_useChinese);
		n.rect = QRectF(0, 0, kNodeW, kNodeH);
		const int index = m_nodes.size();
		m_nodes.push_back(n);
		m_idToIndex.insert(n.id, index);

		for (const auto& child : info.children)
		{
			const int childIndex = addNode(child, false);
			if (childIndex < 0)
			{
				break;
			}
			m_nodes[index].childIndices.push_back(childIndex);
			Edge e;
			e.from = m_nodes[index].id;
			e.to = m_nodes[childIndex].id;
			m_edges.push_back(e);
		}
		return index;
	};

	addNode(root, true);
	if (m_nodes.isEmpty())
	{
		return;
	}
	relayout();
}

void SceneHierarchyCanvasWidget::relayout()
{
	for (Node& n : m_nodes)
	{
		n.visibleInLayout = false;
	}
	if (m_nodes.isEmpty())
	{
		return;
	}
	double yCursor = 0.0;
	layoutSubtree(0, 0, yCursor);
	fitContentInView();
}

double SceneHierarchyCanvasWidget::layoutSubtree(const int nodeIndex, const int depth, double& yCursor)
{
	Node& node = m_nodes[nodeIndex];
	node.visibleInLayout = true;
	const double x = depth * (kNodeW + kHGap);

	if (!node.expanded || node.childIndices.isEmpty())
	{
		node.rect = QRectF(x, yCursor, kNodeW, kNodeH);
		const double centerY = yCursor + kNodeH * 0.5;
		yCursor += kNodeH + kVGap;
		return centerY;
	}

	QVector<double> childCenters;
	childCenters.reserve(node.childIndices.size());
	for (const int ci : node.childIndices)
	{
		childCenters.push_back(layoutSubtree(ci, depth + 1, yCursor));
	}
	const double centerY = (childCenters.front() + childCenters.back()) * 0.5;
	node.rect = QRectF(x, centerY - kNodeH * 0.5, kNodeW, kNodeH);
	return centerY;
}

void SceneHierarchyCanvasWidget::fitContentInView()
{
	if (m_nodes.isEmpty() || width() <= 1 || height() <= 1)
	{
		m_zoom = 1.0;
		m_pan = QPointF(12, 12);
		return;
	}

	QRectF bounds;
	for (const Node& n : m_nodes)
	{
		if (!n.visibleInLayout)
		{
			continue;
		}
		bounds = bounds.isNull() ? n.rect : bounds.united(n.rect);
	}
	if (bounds.isNull())
	{
		m_zoom = 1.0;
		m_pan = QPointF(12, 12);
		return;
	}
	bounds.adjust(-24, -24, 24, 48);
	const double tipH = 28.0;
	const double availW = std::max(40.0, static_cast<double>(width()) - 16.0);
	const double availH = std::max(40.0, static_cast<double>(height()) - tipH - 16.0);
	const double zx = availW / std::max(1.0, bounds.width());
	const double zy = availH / std::max(1.0, bounds.height());
	m_zoom = std::clamp(std::min(zx, zy), 0.25, 1.4);
	const QPointF center = bounds.center();
	m_pan = QPointF(width() * 0.5, (height() - tipH) * 0.5) - center * m_zoom;
}

void SceneHierarchyCanvasWidget::setSceneGraph(const cloudsim::core::IRenderView::SceneNodeInfo& root)
{
	rebuildFromRoot(root);
	update();
}

void SceneHierarchyCanvasWidget::showNodeDetailDialog(const Node& node)
{
	const auto& d = node.detail;
	QDialog dlg(this);
	dlg.setWindowTitle(m_useChinese ? QStringLiteral("节点属性") : QStringLiteral("Node Properties"));
	dlg.resize(480, 560);

	auto* root = new QVBoxLayout(&dlg);
	auto* form = new QFormLayout;
	form->setLabelAlignment(Qt::AlignRight | Qt::AlignVCenter);
	form->setFormAlignment(Qt::AlignTop);
	form->setHorizontalSpacing(12);
	form->setVerticalSpacing(8);

	auto addRow = [&](const QString& zh, const QString& en, const QString& value)
	{
		auto* lab = new QLabel(value.isEmpty() ? QStringLiteral("—") : value, &dlg);
		lab->setTextInteractionFlags(Qt::TextSelectableByMouse);
		lab->setWordWrap(true);
		form->addRow(m_useChinese ? zh : en, lab);
	};

	addRow(QStringLiteral("显示名"), QStringLiteral("Display name"), node.title);
	addRow(QStringLiteral("OSG 名"), QStringLiteral("OSG name"), d.name);
	addRow(QStringLiteral("OSG 类型"), QStringLiteral("OSG class"),
		   OsgSceneNodeI18n::translateClassName(d.className, m_useChinese));
	addRow(QStringLiteral("后端 ID"), QStringLiteral("Backend ID"), d.backendId);
	addRow(QStringLiteral("后端类型"), QStringLiteral("Backend class"),
		   OsgSceneNodeI18n::translateClassName(d.backendClassName, m_useChinese));
	addRow(QStringLiteral("可见"), QStringLiteral("Visible"),
		   d.visible ? (m_useChinese ? QStringLiteral("是") : QStringLiteral("Yes"))
					 : (m_useChinese ? QStringLiteral("否") : QStringLiteral("No")));
	addRow(QStringLiteral("子节点数"), QStringLiteral("Children"), QString::number(d.childCount));
	addRow(QStringLiteral("Drawable"), QStringLiteral("Drawables"), QString::number(d.drawableCount));
	addRow(QStringLiteral("三角面约计"), QStringLiteral("Triangles (approx)"), QString::number(d.triangleCount));
	addRow(QStringLiteral("包围球"), QStringLiteral("Bound"), d.boundSummary);
	addRow(QStringLiteral("渲染"), QStringLiteral("Render"), d.renderSummary);

	form->addRow(m_useChinese ? QStringLiteral("本地矩阵") : QStringLiteral("Local matrix"),
				 makeMonoBlock(&dlg, d.localMatrixSummary));
	form->addRow(m_useChinese ? QStringLiteral("世界矩阵") : QStringLiteral("World matrix"),
				 makeMonoBlock(&dlg, d.worldMatrixSummary));

	auto* scroll = new QScrollArea(&dlg);
	scroll->setWidgetResizable(true);
	scroll->setFrameShape(QFrame::NoFrame);
	auto* formHost = new QWidget;
	formHost->setLayout(form);
	scroll->setWidget(formHost);
	root->addWidget(scroll, 1);

	auto* buttons = new QDialogButtonBox(QDialogButtonBox::Ok, &dlg);
	connect(buttons, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
	root->addWidget(buttons);
	dlg.exec();
}

void SceneHierarchyCanvasWidget::paintEvent(QPaintEvent*)
{
	QPainter p(this);
	p.fillRect(rect(), QColor(QStringLiteral("#EEF1F5")));
	p.setRenderHint(QPainter::Antialiasing, true);

	if (m_nodes.isEmpty())
	{
		p.setPen(QColor(QStringLiteral("#6B7280")));
		const QString text =
			m_emptyHint.isEmpty()
				? (m_useChinese ? QStringLiteral("无场景") : QStringLiteral("No scene"))
				: m_emptyHint;
		p.drawText(rect().adjusted(12, 12, -12, -40), Qt::AlignCenter | Qt::TextWordWrap, text);
	}
	else
	{
		p.save();
		p.translate(m_pan);
		p.scale(m_zoom, m_zoom);

		for (const Edge& e : m_edges)
		{
			const auto fi = m_idToIndex.constFind(e.from);
			const auto ti = m_idToIndex.constFind(e.to);
			if (fi == m_idToIndex.constEnd() || ti == m_idToIndex.constEnd())
			{
				continue;
			}
			const Node& from = m_nodes[fi.value()];
			const Node& to = m_nodes[ti.value()];
			if (!from.visibleInLayout || !to.visibleInLayout || !from.expanded)
			{
				continue;
			}
			const QPointF a = portPos(from, true);
			const QPointF b = portPos(to, false);
			p.setBrush(Qt::NoBrush);
			p.setPen(QPen(QColor(QStringLiteral("#0066CC")), 2.0 / m_zoom));
			QPainterPath path;
			path.moveTo(a);
			const double dx = (b.x() - a.x()) * 0.45;
			path.cubicTo(a + QPointF(dx, 0), b - QPointF(dx, 0), b);
			p.drawPath(path);
		}

		for (const Node& n : m_nodes)
		{
			if (!n.visibleInLayout)
			{
				continue;
			}
			const bool sel = n.id == m_selectedId;
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
			const double titleRightPad = n.childIndices.isEmpty() ? 10.0 : (kExpandBtn + 14.0);
			p.drawText(n.rect.adjusted(10, 8, -titleRightPad, -28), Qt::AlignLeft | Qt::AlignTop | Qt::TextWordWrap,
					   n.title);

			f.setBold(false);
			f.setPointSizeF(8);
			p.setFont(f);
			p.setPen(QColor(QStringLiteral("#6B7280")));
			QString sub = n.subtitle;
			if (!n.childIndices.isEmpty() && !n.expanded)
			{
				sub += QStringLiteral(" · +%1").arg(n.childIndices.size());
			}
			p.drawText(n.rect.adjusted(10, 30, -titleRightPad, -8), Qt::AlignLeft | Qt::AlignTop, sub);

			if (n.isRoot || n.hasBackend)
			{
				const QString badgeText = n.isRoot ? (m_useChinese ? QStringLiteral("根") : QStringLiteral("Root"))
												   : (m_useChinese ? QStringLiteral("对象") : QStringLiteral("Obj"));
				QRectF badge(n.rect.right() - 48, n.rect.top() + 6, 40, 16);
				p.setBrush(n.isRoot ? QColor(QStringLiteral("#1F9D63")) : QColor(QStringLiteral("#0066CC")));
				p.setPen(Qt::NoPen);
				p.drawRoundedRect(badge, 6, 6);
				p.setPen(Qt::white);
				p.drawText(badge, Qt::AlignCenter, badgeText);
			}

			p.setBrush(sel ? QColor(QStringLiteral("#0066CC")) : QColor(QStringLiteral("#4B5563")));
			p.setPen(QPen(QColor(QStringLiteral("#FFFFFF")), 1.2 / m_zoom));
			p.drawEllipse(portPos(n, false), 5, 5);
			if (n.childIndices.isEmpty())
			{
				p.drawEllipse(portPos(n, true), 5, 5);
			}
			else
			{
				// ▸ / ▾ 展开折叠
				const QRectF btn = expandButtonRect(n);
				p.setBrush(QColor(QStringLiteral("#EEF1F5")));
				p.setPen(QPen(QColor(QStringLiteral("#6B7280")), 1.0 / m_zoom));
				p.drawRoundedRect(btn, 4, 4);
				p.setPen(QColor(QStringLiteral("#1C1C1E")));
				QFont bf = font();
				bf.setBold(true);
				bf.setPointSizeF(9);
				p.setFont(bf);
				p.drawText(btn, Qt::AlignCenter, n.expanded ? QStringLiteral("▾") : QStringLiteral("▸"));
			}
		}
		p.restore();
	}

	QString tip = m_useChinese ? QStringLiteral("▸展开/折叠 · 双击属性 · 滚轮缩放 · Alt+拖动画布")
							   : QStringLiteral("▸ expand/collapse · Double-click details · Wheel · Alt+pan");
	if (m_truncated)
	{
		tip += m_useChinese ? QStringLiteral(" · 已截断显示") : QStringLiteral(" · Truncated");
	}
	if (!m_selectedId.isEmpty())
	{
		const auto it = m_idToIndex.constFind(m_selectedId);
		if (it != m_idToIndex.constEnd())
		{
			const Node& n = m_nodes[it.value()];
			if (n.visibleInLayout && !n.detail.localMatrixSummary.isEmpty() &&
				n.detail.localMatrixSummary != QStringLiteral("—"))
			{
				tip = n.detail.localMatrixSummary.split(QLatin1Char('\n')).value(0);
				if (n.hasBackend && !n.detail.backendId.isEmpty())
				{
					tip = n.detail.backendId + QStringLiteral(" · ") + tip;
				}
			}
		}
	}

	const QRect tipBar(0, height() - 28, width(), 28);
	p.fillRect(tipBar, QColor(255, 255, 255, 210));
	p.setPen(QColor(QStringLiteral("#DADCE0")));
	p.drawLine(0, tipBar.top(), width(), tipBar.top());
	p.setPen(QColor(QStringLiteral("#4B5563")));
	QFont tipFont = font();
	tipFont.setPointSizeF(8);
	p.setFont(tipFont);
	p.drawText(tipBar.adjusted(12, 0, -12, 0), Qt::AlignLeft | Qt::AlignVCenter, tip);
}

void SceneHierarchyCanvasWidget::mousePressEvent(QMouseEvent* event)
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

	const int expandHit = hitExpandButton(scene);
	if (expandHit >= 0)
	{
		m_selectedId = m_nodes[expandHit].id;
		m_dragNode = -1;
		toggleExpanded(expandHit);
		return;
	}

	const int ni = hitNode(scene);
	if (ni >= 0)
	{
		m_selectedId = m_nodes[ni].id;
		m_dragNode = ni;
		m_dragOffset = scene - m_nodes[ni].rect.topLeft();
		update();
		return;
	}
	m_selectedId.clear();
	m_dragNode = -1;
	update();
}

void SceneHierarchyCanvasWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
	if (event->button() != Qt::LeftButton)
	{
		return;
	}
	const QPointF scene = toScene(event->pos());
	if (hitExpandButton(scene) >= 0)
	{
		return;
	}
	const int ni = hitNode(scene);
	if (ni < 0)
	{
		return;
	}
	m_selectedId = m_nodes[ni].id;
	m_dragNode = -1;
	update();
	if (m_nodes[ni].hasBackend && !m_nodes[ni].detail.backendId.isEmpty())
	{
		emit backendNodeActivated(m_nodes[ni].detail.backendId);
	}
	showNodeDetailDialog(m_nodes[ni]);
}

void SceneHierarchyCanvasWidget::mouseMoveEvent(QMouseEvent* event)
{
	if (m_panning)
	{
		const QPointF delta = event->pos() - m_panLast;
		m_pan += delta;
		m_panLast = event->pos();
		update();
		return;
	}
	if (m_dragNode >= 0 && m_dragNode < m_nodes.size() && m_nodes[m_dragNode].visibleInLayout)
	{
		const QPointF scene = toScene(event->pos());
		m_nodes[m_dragNode].rect.moveTopLeft(scene - m_dragOffset);
		update();
	}
}

void SceneHierarchyCanvasWidget::mouseReleaseEvent(QMouseEvent* event)
{
	Q_UNUSED(event);
	m_panning = false;
	m_dragNode = -1;
}

void SceneHierarchyCanvasWidget::wheelEvent(QWheelEvent* event)
{
	const double factor = event->angleDelta().y() > 0 ? 1.1 : (1.0 / 1.1);
	const QPointF viewPos = event->position();
	const QPointF before = toScene(viewPos);
	m_zoom = std::clamp(m_zoom * factor, 0.2, 3.0);
	const QPointF after = toScene(viewPos);
	m_pan += (after - before) * m_zoom;
	update();
}

void SceneHierarchyCanvasWidget::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);
	if (!m_nodes.isEmpty())
	{
		fitContentInView();
	}
}
