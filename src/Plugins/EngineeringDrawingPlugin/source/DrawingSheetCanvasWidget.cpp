/// @file DrawingSheetCanvasWidget.cpp
/// @brief 工程图图幅：布局、拖拽、标注、局部放大、导出

#include "DrawingSheetCanvasWidget.h"

#include "DrawingExport.h"
#include "DrawingSidePanel.h"

#include <QDragEnterEvent>
#include <QDragMoveEvent>
#include <QDropEvent>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QLineF>
#include <QMimeData>
#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QResizeEvent>
#include <QWheelEvent>
#include <QtMath>

#include <algorithm>
#include <cmath>

namespace
{
QVector<QPointF> xyArrayToPoints(const QJsonArray& arr)
{
	QVector<QPointF> pts;
	pts.reserve(arr.size() / 2);
	for (int i = 0; i + 1 < arr.size(); i += 2)
		pts.push_back(QPointF(arr.at(i).toDouble(), arr.at(i + 1).toDouble()));
	return pts;
}

QJsonArray pointsToXyArray(const QVector<QPointF>& pts)
{
	QJsonArray arr;
	for (const QPointF& p : pts)
	{
		arr.append(p.x());
		arr.append(p.y());
	}
	return arr;
}

QRectF boundsOfPolylines(const QVector<DrawingSheetCanvasWidget::Polyline2d>& visible,
						 const QVector<DrawingSheetCanvasWidget::Polyline2d>& hidden)
{
	QVector<QPointF> pts;
	auto collect = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& polys) {
		for (const auto& poly : polys)
			for (const QPointF& p : poly.points)
				if (std::isfinite(p.x()) && std::isfinite(p.y()))
					pts.push_back(p);
	};
	collect(visible);
	collect(hidden);
	if (pts.isEmpty())
		return QRectF(0, 0, 10, 10);

	// 先用全体点估包围盒；若被少数野点撑大，则按分位裁掉后再算
	auto bboxOf = [](const QVector<QPointF>& in) {
		double minX = in[0].x(), maxX = in[0].x(), minY = in[0].y(), maxY = in[0].y();
		for (const QPointF& p : in)
		{
			minX = qMin(minX, p.x());
			maxX = qMax(maxX, p.x());
			minY = qMin(minY, p.y());
			maxY = qMax(maxY, p.y());
		}
		return QRectF(minX, minY, qMax(1e-3, maxX - minX), qMax(1e-3, maxY - minY));
	};

	QRectF box = bboxOf(pts);
	if (pts.size() >= 16)
	{
		QVector<double> xs, ys;
		xs.reserve(pts.size());
		ys.reserve(pts.size());
		for (const QPointF& p : pts)
		{
			xs.push_back(p.x());
			ys.push_back(p.y());
		}
		std::sort(xs.begin(), xs.end());
		std::sort(ys.begin(), ys.end());
		const int lo = qMax(0, pts.size() / 50);
		const int hi = qMin(pts.size() - 1, pts.size() - 1 - lo);
		const double rw = xs[hi] - xs[lo];
		const double rh = ys[hi] - ys[lo];
		// 全量包围盒远大于 98% 分位，说明存在野点
		if (box.width() > rw * 20.0 || box.height() > rh * 20.0)
		{
			const double padX = qMax(1.0, rw * 0.05);
			const double padY = qMax(1.0, rh * 0.05);
			box = QRectF(xs[lo] - padX, ys[lo] - padY, rw + 2 * padX, rh + 2 * padY);
		}
	}

	double w = box.width();
	double h = box.height();
	if (w < 1e-3)
		w = 1.0;
	if (h < 1e-3)
		h = 1.0;
	return QRectF(box.x(), box.y(), w, h).adjusted(-2, -2, 2, 2);
}

QVector<DrawingSheetCanvasWidget::Polyline2d> offsetPolylines(
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& src, const QPointF& delta)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> out;
	out.reserve(src.size());
	for (const auto& poly : src)
	{
		DrawingSheetCanvasWidget::Polyline2d n;
		n.points.reserve(poly.points.size());
		for (const QPointF& p : poly.points)
			n.points.push_back(p + delta);
		out.push_back(n);
	}
	return out;
}

QVector<DrawingSheetCanvasWidget::Polyline2d> scalePolylinesAbout(
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& src, const QPointF& origin, double scale)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> out;
	out.reserve(src.size());
	for (const auto& poly : src)
	{
		DrawingSheetCanvasWidget::Polyline2d n;
		n.points.reserve(poly.points.size());
		for (const QPointF& p : poly.points)
			n.points.push_back(origin + (p - origin) * scale);
		out.push_back(n);
	}
	return out;
}

void normalizeViewLocal(const QVector<DrawingSheetCanvasWidget::Polyline2d>& visIn,
						const QVector<DrawingSheetCanvasWidget::Polyline2d>& hidIn,
						QVector<DrawingSheetCanvasWidget::Polyline2d>& visOut,
						QVector<DrawingSheetCanvasWidget::Polyline2d>& hidOut, double& outW, double& outH)
{
	const QRectF box = boundsOfPolylines(visIn, hidIn);
	const QRectF keep = box.adjusted(-1, -1, 1, 1);
	auto clip = [&](const QVector<DrawingSheetCanvasWidget::Polyline2d>& src) {
		QVector<DrawingSheetCanvasWidget::Polyline2d> out;
		out.reserve(src.size());
		for (const auto& poly : src)
		{
			DrawingSheetCanvasWidget::Polyline2d n;
			n.points.reserve(poly.points.size());
			for (const QPointF& p : poly.points)
			{
				if (!std::isfinite(p.x()) || !std::isfinite(p.y()))
					continue;
				if (!keep.contains(p))
					continue;
				n.points.push_back(p - box.topLeft());
			}
			if (n.points.size() >= 2)
				out.push_back(n);
		}
		return out;
	};
	visOut = clip(visIn);
	hidOut = clip(hidIn);
	outW = box.width();
	outH = box.height();
}

DrawingSheetCanvasWidget::DrawingView placeView(const QString& id, const QString& title, const QString& kind,
												const QPointF& origin, double w, double h,
												const QVector<DrawingSheetCanvasWidget::Polyline2d>& lVis,
												const QVector<DrawingSheetCanvasWidget::Polyline2d>& lHid)
{
	DrawingSheetCanvasWidget::DrawingView view;
	view.id = id;
	view.title = title;
	view.kind = kind;
	view.visible = offsetPolylines(lVis, origin);
	view.hidden = offsetPolylines(lHid, origin);
	view.frame = QRectF(origin.x(), origin.y(), w, h).adjusted(-8, -18, 8, 8);
	return view;
}

} // namespace

DrawingSheetCanvasWidget::DrawingSheetCanvasWidget(QWidget* parent) : QWidget(parent)
{
	setMouseTracking(true);
	setFocusPolicy(Qt::StrongFocus);
	setMinimumSize(200, 200);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	setAcceptDrops(true);
}

void DrawingSheetCanvasWidget::clearSheet()
{
	m_views.clear();
	m_dims.clear();
	m_sketch.clear();
	m_selectedSketchId = -1;
	m_selectedDimIndex = -1;
	m_backendId.clear();
	m_needInitialFit = true;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setViews(const QVector<DrawingView>& views)
{
	m_views = views;
	m_needInitialFit = true;
	emit sheetChanged();
	fitToView();
	update();
}

void DrawingSheetCanvasWidget::setDimensions(const QVector<SheetDimension>& dims)
{
	m_dims = dims;
	emit sheetChanged();
	update();
}

bool DrawingSheetCanvasWidget::isSketchTool(DrawingCanvasTool t) const
{
	return t == DrawingCanvasTool::SketchLine || t == DrawingCanvasTool::SketchRect ||
		   t == DrawingCanvasTool::SketchCircle || t == DrawingCanvasTool::SketchArc ||
		   t == DrawingCanvasTool::SketchSpline;
}

void DrawingSheetCanvasWidget::syncSketchTool()
{
	if (!isSketchTool(m_tool))
	{
		m_sketch.clearTool();
		return;
	}
	SketchToolKind kind = SketchToolKind::Line;
	switch (m_tool)
	{
	case DrawingCanvasTool::SketchLine:
		kind = SketchToolKind::Line;
		break;
	case DrawingCanvasTool::SketchRect:
		kind = SketchToolKind::Rectangle;
		break;
	case DrawingCanvasTool::SketchCircle:
		kind = SketchToolKind::Circle;
		break;
	case DrawingCanvasTool::SketchArc:
		kind = SketchToolKind::Arc;
		break;
	case DrawingCanvasTool::SketchSpline:
		kind = SketchToolKind::Spline;
		break;
	default:
		break;
	}
	m_sketch.setTool(kind);
}

double DrawingSheetCanvasWidget::snapTolMm() const
{
	return qMax(0.5, 8.0 / qMax(1e-6, m_zoom));
}

QVector<QPointF> DrawingSheetCanvasWidget::collectViewSnapPoints() const
{
	QVector<QPointF> pts;
	auto addPoly = [&](const QVector<Polyline2d>& polys) {
		for (const Polyline2d& poly : polys)
		{
			if (poly.points.isEmpty())
				continue;
			pts.push_back(poly.points.first());
			pts.push_back(poly.points.last());
			if (poly.points.size() >= 2)
				pts.push_back((poly.points.first() + poly.points.last()) * 0.5);
		}
	};
	for (const DrawingView& v : m_views)
	{
		addPoly(v.visible);
		addPoly(v.hidden);
	}
	return pts;
}

void DrawingSheetCanvasWidget::setProjectionMethod(DrawingProjectionMethod m)
{
	m_projection = m;
}

void DrawingSheetCanvasWidget::setTool(DrawingCanvasTool tool)
{
	m_tool = tool;
	m_dimPicking = false;
	m_dimPickStep = 0;
	m_dimEntityId = -1;
	m_detailDragging = false;
	m_selectedSketchId = -1;
	m_selectedDimIndex = -1;
	syncSketchTool();
	QString tip = QStringLiteral("选择/拖视图");
	switch (tool)
	{
	case DrawingCanvasTool::LinearDim:
		tip = QStringLiteral("线性尺寸：点两点，再点放置尺寸线");
		break;
	case DrawingCanvasTool::DimRadius:
		tip = QStringLiteral("半径：点圆/弧，再点文字位置");
		break;
	case DrawingCanvasTool::DimDiameter:
		tip = QStringLiteral("直径：点圆/弧，再点文字位置");
		break;
	case DrawingCanvasTool::DetailRegion:
		tip = QStringLiteral("局部放大：拖框选区域");
		break;
	case DrawingCanvasTool::SketchLine:
		tip = QStringLiteral("直线：连续点，右键取消");
		break;
	case DrawingCanvasTool::SketchRect:
		tip = QStringLiteral("矩形：对角两点");
		break;
	case DrawingCanvasTool::SketchCircle:
		tip = QStringLiteral("圆：圆心再半径点");
		break;
	case DrawingCanvasTool::SketchArc:
		tip = QStringLiteral("圆弧：起点、中点、终点");
		break;
	case DrawingCanvasTool::SketchSpline:
		tip = QStringLiteral("样条：连续点，右键完成");
		break;
	case DrawingCanvasTool::SelectEntity:
		tip = QStringLiteral("选择图元/尺寸，Delete 删除");
		break;
	default:
		break;
	}
	emit statusMessage(tip);
	update();
}

void DrawingSheetCanvasWidget::setGridVisible(bool visible)
{
	m_gridVisible = visible;
	update();
}

void DrawingSheetCanvasWidget::clampZoom()
{
	// 大尺寸 STEP 图面可达 1e5+，下限过大会只看到视图间隙
	m_zoom = qBound(1e-4, m_zoom, 64.0);
}

QPointF DrawingSheetCanvasWidget::sceneToWidget(const QPointF& scene) const
{
	return scene * m_zoom + m_panOffset;
}

QPointF DrawingSheetCanvasWidget::widgetToScene(const QPointF& widget) const
{
	return (widget - m_panOffset) / qMax(1e-9, m_zoom);
}

QRectF DrawingSheetCanvasWidget::contentBounds() const
{
	QRectF box;
	for (const DrawingView& v : m_views)
		box = box.isValid() ? box.united(v.frame) : v.frame;
	for (const SheetDimension& d : m_dims)
	{
		box = box.united(QRectF(d.p1, QSizeF(0, 0)));
		box = box.united(QRectF(d.p2, QSizeF(0, 0)));
	}
	for (const SheetSketchPolyline& poly : m_sketch.tessellate())
	{
		for (const QPointF& pt : poly.points)
			box = box.isValid() ? box.united(QRectF(pt, QSizeF(0, 0))) : QRectF(pt, QSizeF(1, 1));
	}
	if (!box.isValid())
		return QRectF(0, 0, 200, 200);
	return box.adjusted(-20, -20, 20, 20);
}

void DrawingSheetCanvasWidget::fitToView()
{
	const QRectF box = contentBounds();
	if (width() < 10 || height() < 10)
	{
		m_needInitialFit = true;
		update();
		return;
	}
	const double sx = (width() - 40.0) / qMax(1.0, box.width());
	const double sy = (height() - 40.0) / qMax(1.0, box.height());
	m_zoom = qBound(1e-4, qMin(sx, sy), 64.0);
	m_panOffset = QPointF(width() * 0.5, height() * 0.5) - box.center() * m_zoom;
	m_needInitialFit = false;
	emit viewChanged(m_zoom);
	update();
}

void DrawingSheetCanvasWidget::zoomIn()
{
	m_zoom *= 1.15;
	clampZoom();
	emit viewChanged(m_zoom);
	update();
}

void DrawingSheetCanvasWidget::zoomOut()
{
	m_zoom /= 1.15;
	clampZoom();
	emit viewChanged(m_zoom);
	update();
}

void DrawingSheetCanvasWidget::resetView()
{
	m_zoom = 1.0;
	m_panOffset = QPointF(40, 40);
	emit viewChanged(m_zoom);
	update();
}

bool DrawingSheetCanvasWidget::exportSvg(const QString& filePath) const
{
	return drawing_export::writeSvg(filePath, m_views, m_dims, m_sketch.tessellate());
}

bool DrawingSheetCanvasWidget::exportDxf(const QString& filePath) const
{
	return drawing_export::writeDxf(filePath, m_views, m_dims, m_sketch.tessellate());
}

void DrawingSheetCanvasWidget::drawGrid(QPainter& p) const
{
	if (!m_gridVisible)
		return;
	p.save();
	p.setPen(QPen(QColor(220, 225, 232), 1));
	const QPointF tl = widgetToScene(QPointF(0, 0));
	const QPointF br = widgetToScene(QPointF(width(), height()));
	const double step = 20.0;
	const int x0 = static_cast<int>(std::floor(tl.x() / step));
	const int x1 = static_cast<int>(std::ceil(br.x() / step));
	const int y0 = static_cast<int>(std::floor(tl.y() / step));
	const int y1 = static_cast<int>(std::ceil(br.y() / step));
	for (int x = x0; x <= x1; ++x)
	{
		const double sx = x * step;
		p.drawLine(sceneToWidget(QPointF(sx, tl.y())), sceneToWidget(QPointF(sx, br.y())));
	}
	for (int y = y0; y <= y1; ++y)
	{
		const double sy = y * step;
		p.drawLine(sceneToWidget(QPointF(tl.x(), sy)), sceneToWidget(QPointF(br.x(), sy)));
	}
	p.restore();
}

void DrawingSheetCanvasWidget::drawView(QPainter& p, const DrawingView& view) const
{
	const QRectF wFrame = QRectF(sceneToWidget(view.frame.topLeft()), sceneToWidget(view.frame.bottomRight())).normalized();
	p.save();
	p.setPen(QPen(QColor(90, 100, 120), 1.5));
	p.setBrush(Qt::NoBrush);
	p.drawRect(wFrame);
	p.setPen(QPen(QColor(50, 60, 80), 1));
	p.drawText(wFrame.adjusted(4, 2, -4, 0).topLeft() + QPointF(0, 12), view.title);

	auto drawPolys = [&](const QVector<Polyline2d>& polys, const QPen& pen) {
		p.setPen(pen);
		for (const Polyline2d& poly : polys)
		{
			if (poly.points.size() < 2)
				continue;
			QPolygonF polyWidget;
			for (const QPointF& pt : poly.points)
				polyWidget << sceneToWidget(pt);
			p.drawPolyline(polyWidget);
		}
	};
	drawPolys(view.hidden, QPen(QColor(140, 145, 155), 1.0, Qt::DashLine));
	drawPolys(view.visible, QPen(QColor(20, 24, 32), 1.6, Qt::SolidLine));
	p.restore();
}

void DrawingSheetCanvasWidget::drawSketch(QPainter& p) const
{
	p.save();
	for (const SheetSketchPolyline& poly : m_sketch.tessellate())
	{
		if (poly.points.size() < 2)
			continue;
		const bool sel = poly.entityId >= 0 && poly.entityId == m_selectedSketchId;
		QPen pen(poly.construction ? QColor(140, 145, 155) : QColor(30, 120, 180), sel ? 2.4 : 1.6,
				 poly.construction ? Qt::DashLine : Qt::SolidLine);
		p.setPen(pen);
		QPolygonF w;
		for (const QPointF& pt : poly.points)
			w << sceneToWidget(pt);
		p.drawPolyline(w);
	}
	const QVector<QPointF> preview = m_sketch.previewPolyline();
	if (preview.size() >= 2)
	{
		p.setPen(QPen(QColor(41, 128, 185), 1.2, Qt::DashLine));
		QPolygonF w;
		for (const QPointF& pt : preview)
			w << sceneToWidget(pt);
		p.drawPolyline(w);
	}
	const SkSnapResult snap = m_sketch.lastSnap();
	if (snap.snapped)
	{
		const QPointF c = sceneToWidget(SheetSketchAdapter::toScene(snap.pos));
		p.setPen(QPen(QColor(230, 126, 34), 1.5));
		p.drawEllipse(c, 5, 5);
	}
	p.restore();
}

void DrawingSheetCanvasWidget::drawDimArrow(QPainter& p, const QPointF& tipWidget, const QPointF& dirScene) const
{
	QLineF dir(QPointF(0, 0), dirScene);
	if (dir.length() < 1e-9)
		return;
	dir.setLength(1.0);
	const QPointF d = dir.p2();
	const QPointF n(-d.y(), d.x());
	const double len = 7.0;
	const QPointF base = tipWidget - d * len;
	QPolygonF tri;
	tri << tipWidget << (base + n * 3.0) << (base - n * 3.0);
	p.setBrush(p.pen().color());
	p.drawPolygon(tri);
	p.setBrush(Qt::NoBrush);
}

double DrawingSheetCanvasWidget::dimensionValue(const SheetDimension& dim) const
{
	if (std::isfinite(dim.overrideValue))
		return dim.overrideValue;
	if (dim.kind == SheetDimension::Kind::Linear)
		return QLineF(dim.p1, dim.p2).length();
	const double r = QLineF(dim.p1, dim.p2).length();
	return dim.kind == SheetDimension::Kind::Diameter ? (2.0 * r) : r;
}

QString DrawingSheetCanvasWidget::dimensionText(const SheetDimension& dim) const
{
	const double v = dimensionValue(dim);
	if (dim.kind == SheetDimension::Kind::Radius)
		return QStringLiteral("R%1").arg(v, 0, 'f', 2);
	if (dim.kind == SheetDimension::Kind::Diameter)
		return QStringLiteral("Ø%1").arg(v, 0, 'f', 2);
	return QString::number(v, 'f', 2);
}

void DrawingSheetCanvasWidget::drawDimension(QPainter& p, const SheetDimension& dim) const
{
	p.save();
	const bool sel = m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size() &&
					 m_dims[m_selectedDimIndex].id == dim.id;
	QPen pen(sel ? QColor(142, 68, 173) : QColor(192, 57, 43), sel ? 1.8 : 1.2);
	p.setPen(pen);

	if (dim.kind == SheetDimension::Kind::Linear)
	{
		QLineF base(dim.p1, dim.p2);
		if (base.length() < 1e-9)
		{
			p.restore();
			return;
		}
		base.setLength(1.0);
		const QPointF dir = base.p2() - base.p1();
		QPointF n(-dir.y(), dir.x());
		// textOffset 的长度决定尺寸线偏移
		double off = dim.textOffset.y();
		if (std::abs(off) < 1e-6)
			off = -12.0;
		const QPointF offset = n * off;
		const QPointF a = dim.p1 + offset;
		const QPointF b = dim.p2 + offset;
		const QPointF wa = sceneToWidget(a);
		const QPointF wb = sceneToWidget(b);
		const QPointF wp1 = sceneToWidget(dim.p1);
		const QPointF wp2 = sceneToWidget(dim.p2);
		p.drawLine(wp1, sceneToWidget(dim.p1 + offset * 1.15));
		p.drawLine(wp2, sceneToWidget(dim.p2 + offset * 1.15));
		p.drawLine(wa, wb);
		drawDimArrow(p, wa, -(b - a));
		drawDimArrow(p, wb, b - a);
		const QPointF mid = (wa + wb) * 0.5 + QPointF(dim.textOffset.x(), 0) * m_zoom;
		p.drawText(mid, dimensionText(dim));
	}
	else
	{
		const QPointF c = sceneToWidget(dim.p1);
		const QPointF r = sceneToWidget(dim.p2);
		const QPointF textPos = sceneToWidget(dim.p2 + dim.textOffset);
		if (dim.kind == SheetDimension::Kind::Diameter)
		{
			const QPointF other = dim.p1 - (dim.p2 - dim.p1);
			p.drawLine(sceneToWidget(other), r);
			drawDimArrow(p, r, dim.p2 - dim.p1);
			drawDimArrow(p, sceneToWidget(other), other - dim.p1);
		}
		else
		{
			p.drawLine(c, r);
			drawDimArrow(p, r, dim.p2 - dim.p1);
		}
		p.drawText(textPos, dimensionText(dim));
	}
	p.restore();
}

void DrawingSheetCanvasWidget::paintEvent(QPaintEvent*)
{
	// 与工艺流程一致：首次有内容时在 paint 里补 fit，避免仅靠 resize
	if (m_needInitialFit && !isEmpty() && width() >= 10 && height() >= 10)
		fitToView();

	QPainter p(this);
	p.fillRect(rect(), QColor(0xF5, 0xF7, 0xFA));
	p.setRenderHint(QPainter::Antialiasing, true);
	drawGrid(p);
	for (const DrawingView& v : m_views)
		drawView(p, v);
	drawSketch(p);
	for (const SheetDimension& d : m_dims)
		drawDimension(p, d);
	if (m_detailDragging)
	{
		p.setPen(QPen(QColor(41, 128, 185), 1.2, Qt::DashLine));
		p.setBrush(QColor(41, 128, 185, 40));
		const QRectF r = QRectF(sceneToWidget(m_detailStart), sceneToWidget(m_detailCurrent)).normalized();
		p.drawRect(r);
	}
	if (m_dimPicking && m_tool == DrawingCanvasTool::LinearDim && m_dimPickStep >= 1)
	{
		p.setPen(QPen(QColor(192, 57, 43), 1.0, Qt::DashLine));
		if (m_dimPickStep == 1)
			p.drawLine(sceneToWidget(m_dimP1), sceneToWidget(m_dimP2));
		else if (m_dimPickStep == 2)
			p.drawLine(sceneToWidget(m_dimP1), sceneToWidget(m_dimP2));
	}
}

void DrawingSheetCanvasWidget::resizeEvent(QResizeEvent* event)
{
	QWidget::resizeEvent(event);
	if (m_needInitialFit && !m_views.isEmpty())
		fitToView();
}

int DrawingSheetCanvasWidget::hitViewIndex(const QPointF& scenePos) const
{
	for (int i = m_views.size() - 1; i >= 0; --i)
	{
		if (m_views[i].frame.contains(scenePos))
			return i;
	}
	return -1;
}

void DrawingSheetCanvasWidget::moveViewBy(int index, const QPointF& deltaScene)
{
	if (index < 0 || index >= m_views.size())
		return;
	DrawingView& v = m_views[index];
	v.frame.translate(deltaScene);
	for (Polyline2d& poly : v.visible)
		for (QPointF& pt : poly.points)
			pt += deltaScene;
	for (Polyline2d& poly : v.hidden)
		for (QPointF& pt : poly.points)
			pt += deltaScene;
	emit sheetChanged();
	update();
}

bool DrawingSheetCanvasWidget::addDetailView(const QString& parentViewId, const QRectF& regionScene, double scale)
{
	int parentIdx = -1;
	for (int i = 0; i < m_views.size(); ++i)
	{
		if (m_views[i].id == parentViewId)
		{
			parentIdx = i;
			break;
		}
	}
	if (parentIdx < 0)
		return false;
	const DrawingView& parent = m_views[parentIdx];
	const QRectF region = regionScene.normalized();
	if (region.width() < 1e-3 || region.height() < 1e-3)
		return false;

	auto clipScale = [&](const QVector<Polyline2d>& src) {
		QVector<Polyline2d> out;
		for (const Polyline2d& poly : src)
		{
			Polyline2d n;
			for (const QPointF& p : poly.points)
			{
				if (region.contains(p))
					n.points.push_back((p - region.topLeft()) * scale);
			}
			if (n.points.size() >= 2)
				out.push_back(n);
		}
		return out;
	};

	DrawingView detail;
	detail.id = QStringLiteral("detail_%1").arg(m_nextDetailId++);
	detail.title = QStringLiteral("局部 ×%1").arg(scale, 0, 'f', 1);
	detail.kind = QStringLiteral("detail");
	detail.parentViewId = parentViewId;
	detail.contentScale = scale;
	detail.visible = clipScale(parent.visible);
	detail.hidden = clipScale(parent.hidden);
	const QRectF localBox = boundsOfPolylines(detail.visible, detail.hidden);
	detail.visible = offsetPolylines(detail.visible, -localBox.topLeft());
	detail.hidden = offsetPolylines(detail.hidden, -localBox.topLeft());
	const QPointF origin(contentBounds().right() + 40.0, contentBounds().top());
	detail.visible = offsetPolylines(detail.visible, origin);
	detail.hidden = offsetPolylines(detail.hidden, origin);
	detail.frame = QRectF(origin, localBox.size()).adjusted(-8, -18, 8, 8);
	m_views.push_back(detail);
	emit sheetChanged();
	fitToView();
	return true;
}

bool DrawingSheetCanvasWidget::resolveCircleDim(int entityId, QPointF& center, QPointF& rim, double& radius) const
{
	if (const SkCircle* c = m_sketch.document().findCircle(entityId))
	{
		const SkPoint* cen = m_sketch.document().findPoint(c->center);
		if (!cen)
			return false;
		center = SheetSketchAdapter::toScene(cen->p);
		radius = c->radius;
		rim = QPointF(center.x() + radius, center.y());
		return true;
	}
	if (const SkArc* arc = m_sketch.document().findArc(entityId))
	{
		const SkPoint* s = m_sketch.document().findPoint(arc->pStart);
		const SkPoint* m = m_sketch.document().findPoint(arc->pMid);
		const SkPoint* e = m_sketch.document().findPoint(arc->pEnd);
		if (!s || !m || !e)
			return false;
		SkVec2 cen;
		if (!sketchCircumcenter(s->p, m->p, e->p, cen, radius))
			return false;
		center = SheetSketchAdapter::toScene(cen);
		rim = SheetSketchAdapter::toScene(s->p);
		return true;
	}
	return false;
}

int DrawingSheetCanvasWidget::hitDimensionIndex(const QPointF& scenePos) const
{
	const double tol = snapTolMm();
	for (int i = m_dims.size() - 1; i >= 0; --i)
	{
		const SheetDimension& d = m_dims[i];
		if (d.kind == SheetDimension::Kind::Linear)
		{
			QLineF base(d.p1, d.p2);
			if (base.length() < 1e-9)
				continue;
			base.setLength(1.0);
			const QPointF dir = base.p2() - base.p1();
			const QPointF n(-dir.y(), dir.x());
			double off = d.textOffset.y();
			if (std::abs(off) < 1e-6)
				off = -12.0;
			const QLineF dimLine(d.p1 + n * off, d.p2 + n * off);
			const QPointF a = dimLine.p1();
			const QPointF b = dimLine.p2();
			const QPointF ab = b - a;
			const double len2 = QPointF::dotProduct(ab, ab);
			double t = len2 > 1e-12 ? QPointF::dotProduct(scenePos - a, ab) / len2 : 0.0;
			t = qBound(0.0, t, 1.0);
			const QPointF proj = a + ab * t;
			if (QLineF(proj, scenePos).length() <= tol)
				return i;
		}
		else
		{
			if (QLineF(d.p1, scenePos).length() <= tol || QLineF(d.p2, scenePos).length() <= tol ||
				QLineF(d.p2 + d.textOffset, scenePos).length() <= tol * 2)
				return i;
			QLineF ray(d.p1, d.p2);
			const QPointF a = ray.p1();
			const QPointF b = ray.p2();
			const QPointF ab = b - a;
			const double len2 = QPointF::dotProduct(ab, ab);
			double t = len2 > 1e-12 ? QPointF::dotProduct(scenePos - a, ab) / len2 : 0.0;
			t = qBound(0.0, t, 1.0);
			if (QLineF(a + ab * t, scenePos).length() <= tol)
				return i;
		}
	}
	return -1;
}

void DrawingSheetCanvasWidget::mousePressEvent(QMouseEvent* event)
{
	const QPointF widgetPos = event->pos();
	const QPointF scenePos = widgetToScene(widgetPos);
	const QVector<QPointF> extraSnap = collectViewSnapPoints();

	if (event->button() == Qt::MiddleButton ||
		(event->button() == Qt::LeftButton && (event->modifiers() & Qt::AltModifier)))
	{
		m_panning = true;
		m_lastWidgetPos = widgetPos;
		event->accept();
		return;
	}

	if (event->button() == Qt::RightButton && isSketchTool(m_tool))
	{
		m_sketch.press(scenePos, true, snapTolMm(), extraSnap);
		emit sheetChanged();
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && isSketchTool(m_tool))
	{
		m_sketch.press(scenePos, false, snapTolMm(), extraSnap);
		emit sheetChanged();
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::LinearDim)
	{
		const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
		const QPointF sp = SheetSketchAdapter::toScene(snapped);
		if (!m_dimPicking)
		{
			m_dimP1 = sp;
			m_dimP2 = sp;
			m_dimPicking = true;
			m_dimPickStep = 1;
			emit statusMessage(QStringLiteral("尺寸：再点第二点"));
		}
		else if (m_dimPickStep == 1)
		{
			m_dimP2 = sp;
			m_dimPickStep = 2;
			emit statusMessage(QStringLiteral("尺寸：再点放置尺寸线"));
		}
		else
		{
			SheetDimension dim;
			dim.kind = SheetDimension::Kind::Linear;
			dim.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
			dim.p1 = m_dimP1;
			dim.p2 = m_dimP2;
			QLineF base(m_dimP1, m_dimP2);
			if (base.length() > 1e-9)
			{
				base.setLength(1.0);
				const QPointF dir = base.p2() - base.p1();
				const QPointF n(-dir.y(), dir.x());
				const QPointF mid = (m_dimP1 + m_dimP2) * 0.5;
				dim.textOffset = QPointF(0.0, QPointF::dotProduct(sp - mid, n));
				if (std::abs(dim.textOffset.y()) < 1.0)
					dim.textOffset.setY(-12.0);
			}
			m_dims.push_back(dim);
			m_dimPicking = false;
			m_dimPickStep = 0;
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已添加线性尺寸"));
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton &&
		(m_tool == DrawingCanvasTool::DimRadius || m_tool == DrawingCanvasTool::DimDiameter))
	{
		if (!m_dimPicking)
		{
			const int ent = m_sketch.hitTestEntity(scenePos, snapTolMm());
			QPointF c, rim;
			double r = 0;
			if (ent < 0 || !resolveCircleDim(ent, c, rim, r))
			{
				emit statusMessage(QStringLiteral("请点选圆或圆弧"));
				event->accept();
				return;
			}
			m_dimEntityId = ent;
			m_dimP1 = c;
			// 半径方向取点击点
			QLineF ray(c, scenePos);
			if (ray.length() < 1e-6)
				ray = QLineF(c, rim);
			ray.setLength(r);
			m_dimP2 = ray.p2();
			m_dimPicking = true;
			m_dimPickStep = 1;
			emit statusMessage(QStringLiteral("再点文字位置"));
		}
		else
		{
			SheetDimension dim;
			dim.kind = (m_tool == DrawingCanvasTool::DimDiameter) ? SheetDimension::Kind::Diameter
																 : SheetDimension::Kind::Radius;
			dim.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
			dim.p1 = m_dimP1;
			dim.p2 = m_dimP2;
			dim.sketchEntityId = m_dimEntityId;
			dim.textOffset = scenePos - m_dimP2;
			m_dims.push_back(dim);
			m_dimPicking = false;
			m_dimPickStep = 0;
			m_dimEntityId = -1;
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已添加尺寸"));
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::SelectEntity)
	{
		m_selectedDimIndex = hitDimensionIndex(scenePos);
		m_selectedSketchId = -1;
		if (m_selectedDimIndex < 0)
			m_selectedSketchId = m_sketch.hitTestEntity(scenePos, snapTolMm());
		emit statusMessage(m_selectedDimIndex >= 0 || m_selectedSketchId >= 0
							   ? QStringLiteral("已选中，按 Delete 删除")
							   : QStringLiteral("未命中"));
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::DetailRegion)
	{
		m_detailDragging = true;
		m_detailStart = m_detailCurrent = scenePos;
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::PanSelect)
	{
		const int hit = hitViewIndex(scenePos);
		if (hit >= 0)
		{
			m_draggingView = true;
			m_dragViewIndex = hit;
			m_lastWidgetPos = widgetPos;
			event->accept();
			return;
		}
	}

	QWidget::mousePressEvent(event);
}

void DrawingSheetCanvasWidget::mouseMoveEvent(QMouseEvent* event)
{
	const QPointF widgetPos = event->pos();
	const QPointF scenePos = widgetToScene(widgetPos);
	if (m_panning)
	{
		m_panOffset += widgetPos - m_lastWidgetPos;
		m_lastWidgetPos = widgetPos;
		emit viewChanged(m_zoom);
		update();
		event->accept();
		return;
	}
	if (m_draggingView && m_dragViewIndex >= 0)
	{
		const QPointF deltaWidget = widgetPos - m_lastWidgetPos;
		m_lastWidgetPos = widgetPos;
		moveViewBy(m_dragViewIndex, deltaWidget / qMax(0.001, m_zoom));
		event->accept();
		return;
	}
	if (m_detailDragging)
	{
		m_detailCurrent = scenePos;
		update();
		event->accept();
		return;
	}
	if (isSketchTool(m_tool))
	{
		m_sketch.move(scenePos, snapTolMm(), collectViewSnapPoints());
		update();
		event->accept();
		return;
	}
	if (m_dimPicking && m_tool == DrawingCanvasTool::LinearDim && m_dimPickStep == 1)
	{
		const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), collectViewSnapPoints(), nullptr);
		m_dimP2 = SheetSketchAdapter::toScene(snapped);
		update();
		event->accept();
		return;
	}
	QWidget::mouseMoveEvent(event);
}

void DrawingSheetCanvasWidget::mouseReleaseEvent(QMouseEvent* event)
{
	if (m_panning && (event->button() == Qt::MiddleButton || event->button() == Qt::LeftButton))
	{
		m_panning = false;
		event->accept();
		return;
	}
	if (m_draggingView && event->button() == Qt::LeftButton)
	{
		m_draggingView = false;
		m_dragViewIndex = -1;
		event->accept();
		return;
	}
	if (m_detailDragging && event->button() == Qt::LeftButton)
	{
		m_detailDragging = false;
		const QRectF region = QRectF(m_detailStart, m_detailCurrent).normalized();
		int parentIdx = hitViewIndex(region.center());
		if (parentIdx < 0)
			parentIdx = hitViewIndex(m_detailStart);
		if (parentIdx >= 0)
		{
			addDetailView(m_views[parentIdx].id, region, 2.0);
			emit statusMessage(QStringLiteral("已添加局部放大视图"));
		}
		else
		{
			emit statusMessage(QStringLiteral("未命中父视图"));
		}
		update();
		event->accept();
		return;
	}
	QWidget::mouseReleaseEvent(event);
}

void DrawingSheetCanvasWidget::mouseDoubleClickEvent(QMouseEvent* event)
{
	if (event->button() != Qt::LeftButton)
	{
		QWidget::mouseDoubleClickEvent(event);
		return;
	}
	const int idx = hitDimensionIndex(widgetToScene(event->pos()));
	if (idx < 0)
	{
		QWidget::mouseDoubleClickEvent(event);
		return;
	}
	SheetDimension& dim = m_dims[idx];
	bool ok = false;
	const double cur = dimensionValue(dim);
	const double v = QInputDialog::getDouble(this, QStringLiteral("编辑尺寸"), QStringLiteral("显示值"), cur, -1e9,
											 1e9, 3, &ok);
	if (ok)
	{
		dim.overrideValue = v;
		emit sheetChanged();
		update();
	}
	event->accept();
}

void DrawingSheetCanvasWidget::keyPressEvent(QKeyEvent* event)
{
	if (event->key() == Qt::Key_Escape)
	{
		m_sketch.cancelTool();
		m_dimPicking = false;
		m_dimPickStep = 0;
		m_selectedSketchId = -1;
		m_selectedDimIndex = -1;
		update();
		event->accept();
		return;
	}
	if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace)
	{
		bool changed = false;
		if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
		{
			m_dims.removeAt(m_selectedDimIndex);
			m_selectedDimIndex = -1;
			changed = true;
		}
		else if (m_selectedSketchId >= 0)
		{
			changed = m_sketch.removeEntity(m_selectedSketchId);
			m_selectedSketchId = -1;
		}
		if (changed)
		{
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已删除"));
			update();
		}
		event->accept();
		return;
	}
	QWidget::keyPressEvent(event);
}

void DrawingSheetCanvasWidget::wheelEvent(QWheelEvent* event)
{
	const QPointF widgetPoint = event->position();
	const QPointF scenePoint = widgetToScene(widgetPoint);
	m_zoom *= (event->angleDelta().y() > 0 ? 1.15 : 1.0 / 1.15);
	clampZoom();
	m_panOffset = widgetPoint - scenePoint * m_zoom;
	emit viewChanged(m_zoom);
	update();
	event->accept();
}

QJsonObject DrawingSheetCanvasWidget::toJson() const
{
	QJsonObject root;
	root.insert(QStringLiteral("version"), 3);
	root.insert(QStringLiteral("backendId"), m_backendId);
	root.insert(QStringLiteral("projection"),
				m_projection == DrawingProjectionMethod::ThirdAngle ? QStringLiteral("thirdAngle")
																	: QStringLiteral("firstAngle"));
	QJsonArray viewsArr;
	for (const DrawingView& v : m_views)
	{
		QJsonObject vo;
		vo.insert(QStringLiteral("id"), v.id);
		vo.insert(QStringLiteral("title"), v.title);
		vo.insert(QStringLiteral("kind"), v.kind);
		vo.insert(QStringLiteral("parentViewId"), v.parentViewId);
		vo.insert(QStringLiteral("contentScale"), v.contentScale);
		vo.insert(QStringLiteral("frame"), QJsonArray{v.frame.x(), v.frame.y(), v.frame.width(), v.frame.height()});
		QJsonArray vis;
		for (const Polyline2d& poly : v.visible)
			vis.append(pointsToXyArray(poly.points));
		QJsonArray hid;
		for (const Polyline2d& poly : v.hidden)
			hid.append(pointsToXyArray(poly.points));
		vo.insert(QStringLiteral("visible"), vis);
		vo.insert(QStringLiteral("hidden"), hid);
		viewsArr.append(vo);
	}
	root.insert(QStringLiteral("views"), viewsArr);
	QJsonArray dimsArr;
	for (const SheetDimension& d : m_dims)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), d.id);
		QString kind = QStringLiteral("linear");
		if (d.kind == SheetDimension::Kind::Radius)
			kind = QStringLiteral("radius");
		else if (d.kind == SheetDimension::Kind::Diameter)
			kind = QStringLiteral("diameter");
		o.insert(QStringLiteral("kind"), kind);
		o.insert(QStringLiteral("p1"), QJsonArray{d.p1.x(), d.p1.y()});
		o.insert(QStringLiteral("p2"), QJsonArray{d.p2.x(), d.p2.y()});
		o.insert(QStringLiteral("textOffset"), QJsonArray{d.textOffset.x(), d.textOffset.y()});
		o.insert(QStringLiteral("sketchEntityId"), d.sketchEntityId);
		if (std::isfinite(d.overrideValue))
			o.insert(QStringLiteral("overrideValue"), d.overrideValue);
		dimsArr.append(o);
	}
	root.insert(QStringLiteral("dimensions"), dimsArr);
	const QByteArray sketchUtf8 = m_sketch.toJsonUtf8();
	if (!sketchUtf8.isEmpty())
	{
		const QJsonDocument sd = QJsonDocument::fromJson(sketchUtf8);
		if (sd.isObject())
			root.insert(QStringLiteral("sketch"), sd.object());
	}
	return root;
}

bool DrawingSheetCanvasWidget::fromJson(const QJsonObject& root)
{
	m_views.clear();
	m_dims.clear();
	m_sketch.clear();
	m_selectedSketchId = -1;
	m_selectedDimIndex = -1;
	m_backendId = root.value(QStringLiteral("backendId")).toString();
	const QString proj = root.value(QStringLiteral("projection")).toString();
	m_projection = (proj == QLatin1String("thirdAngle")) ? DrawingProjectionMethod::ThirdAngle
														 : DrawingProjectionMethod::FirstAngle;
	for (const QJsonValue& vv : root.value(QStringLiteral("views")).toArray())
	{
		const QJsonObject vo = vv.toObject();
		DrawingView view;
		view.id = vo.value(QStringLiteral("id")).toString();
		view.title = vo.value(QStringLiteral("title")).toString(view.id);
		view.kind = vo.value(QStringLiteral("kind")).toString(QStringLiteral("front"));
		view.parentViewId = vo.value(QStringLiteral("parentViewId")).toString();
		view.contentScale = vo.value(QStringLiteral("contentScale")).toDouble(1.0);
		const QJsonArray fr = vo.value(QStringLiteral("frame")).toArray();
		if (fr.size() >= 4)
			view.frame = QRectF(fr.at(0).toDouble(), fr.at(1).toDouble(), fr.at(2).toDouble(), fr.at(3).toDouble());
		for (const QJsonValue& pv : vo.value(QStringLiteral("visible")).toArray())
		{
			Polyline2d poly;
			poly.points = xyArrayToPoints(pv.toArray());
			if (poly.points.size() >= 2)
				view.visible.push_back(poly);
		}
		for (const QJsonValue& pv : vo.value(QStringLiteral("hidden")).toArray())
		{
			Polyline2d poly;
			poly.points = xyArrayToPoints(pv.toArray());
			if (poly.points.size() >= 2)
				view.hidden.push_back(poly);
		}
		m_views.push_back(view);
	}
	for (const QJsonValue& dv : root.value(QStringLiteral("dimensions")).toArray())
	{
		const QJsonObject o = dv.toObject();
		SheetDimension dim;
		dim.id = o.value(QStringLiteral("id")).toString();
		const QString kind = o.value(QStringLiteral("kind")).toString(QStringLiteral("linear"));
		if (kind == QLatin1String("radius"))
			dim.kind = SheetDimension::Kind::Radius;
		else if (kind == QLatin1String("diameter"))
			dim.kind = SheetDimension::Kind::Diameter;
		else
			dim.kind = SheetDimension::Kind::Linear;
		const QJsonArray a = o.value(QStringLiteral("p1")).toArray();
		const QJsonArray b = o.value(QStringLiteral("p2")).toArray();
		if (a.size() >= 2)
			dim.p1 = QPointF(a.at(0).toDouble(), a.at(1).toDouble());
		if (b.size() >= 2)
			dim.p2 = QPointF(b.at(0).toDouble(), b.at(1).toDouble());
		const QJsonArray to = o.value(QStringLiteral("textOffset")).toArray();
		if (to.size() >= 2)
			dim.textOffset = QPointF(to.at(0).toDouble(), to.at(1).toDouble());
		dim.sketchEntityId = o.value(QStringLiteral("sketchEntityId")).toInt(-1);
		if (o.contains(QStringLiteral("overrideValue")))
			dim.overrideValue = o.value(QStringLiteral("overrideValue")).toDouble();
		m_dims.push_back(dim);
	}
	if (root.contains(QStringLiteral("sketch")))
	{
		const QJsonObject so = root.value(QStringLiteral("sketch")).toObject();
		m_sketch.fromJsonUtf8(QJsonDocument(so).toJson(QJsonDocument::Compact));
	}
	emit sheetChanged();
	m_needInitialFit = true;
	fitToView();
	return true;
}

void DrawingSheetCanvasWidget::setViewCatalog(const QVector<ViewTemplate>& catalog)
{
	m_viewCatalog = catalog;
}

bool DrawingSheetCanvasWidget::addCatalogViewAt(const QString& kind, const QPointF& sceneTopLeft)
{
	const ViewTemplate* found = nullptr;
	for (const ViewTemplate& t : m_viewCatalog)
	{
		if (t.kind == kind)
		{
			found = &t;
			break;
		}
	}
	if (!found || (found->visible.isEmpty() && found->hidden.isEmpty()))
		return false;

	QVector<Polyline2d> lVis, lHid;
	double w = 0, h = 0;
	normalizeViewLocal(found->visible, found->hidden, lVis, lHid, w, h);
	DrawingView view;
	view.id = QStringLiteral("%1_%2").arg(kind).arg(m_nextCatalogViewId++);
	view.title = found->title.isEmpty() ? kind : found->title;
	view.kind = kind;
	view.visible = offsetPolylines(lVis, sceneTopLeft);
	view.hidden = offsetPolylines(lHid, sceneTopLeft);
	view.frame = QRectF(sceneTopLeft.x(), sceneTopLeft.y(), w, h).adjusted(-8, -18, 8, 8);
	m_views.push_back(view);
	emit sheetChanged();
	emit statusMessage(QStringLiteral("已添加%1").arg(view.title));
	update();
	return true;
}

void DrawingSheetCanvasWidget::dragEnterEvent(QDragEnterEvent* event)
{
	if (event->mimeData() && event->mimeData()->hasFormat(QString::fromLatin1(drawingViewMimeType())))
		event->acceptProposedAction();
}

void DrawingSheetCanvasWidget::dragMoveEvent(QDragMoveEvent* event)
{
	if (event->mimeData() && event->mimeData()->hasFormat(QString::fromLatin1(drawingViewMimeType())))
		event->acceptProposedAction();
}

void DrawingSheetCanvasWidget::dropEvent(QDropEvent* event)
{
	if (!event->mimeData() || !event->mimeData()->hasFormat(QString::fromLatin1(drawingViewMimeType())))
		return;
	const QJsonObject o =
		QJsonDocument::fromJson(event->mimeData()->data(QString::fromLatin1(drawingViewMimeType()))).object();
	const QString kind = o.value(QStringLiteral("kind")).toString();
	if (kind.isEmpty())
		return;
	const QPointF scenePos = widgetToScene(event->pos());
	if (addCatalogViewAt(kind, scenePos))
		event->acceptProposedAction();
}

QVector<DrawingSheetCanvasWidget::DrawingView> layoutEngineeringViews(
	DrawingProjectionMethod method, bool hasIso, bool hasSection,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& frontVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& frontHid,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& topVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& topHid,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& rightVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& rightHid,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& isoVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& isoHid,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& sectionVis,
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& sectionHid)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> topLVis, topLHid, frontLVis, frontLHid, rightLVis, rightLHid;
	QVector<DrawingSheetCanvasWidget::Polyline2d> isoLVis, isoLHid, secLVis, secLHid;
	double topW = 0, topH = 0, frontW = 0, frontH = 0, rightW = 0, rightH = 0;
	double isoW = 0, isoH = 0, secW = 0, secH = 0;
	normalizeViewLocal(topVis, topHid, topLVis, topLHid, topW, topH);
	normalizeViewLocal(frontVis, frontHid, frontLVis, frontLHid, frontW, frontH);
	normalizeViewLocal(rightVis, rightHid, rightLVis, rightLHid, rightW, rightH);
	if (hasIso)
		normalizeViewLocal(isoVis, isoHid, isoLVis, isoLHid, isoW, isoH);
	if (hasSection)
		normalizeViewLocal(sectionVis, sectionHid, secLVis, secLHid, secW, secH);

	const double extent = qMax(qMax(topW, topH), qMax(qMax(frontW, frontH), qMax(rightW, rightH)));
	const double gap = qMax(30.0, extent * 0.2);
	const double col0W = qMax(topW, frontW);

	QVector<DrawingSheetCanvasWidget::DrawingView> views;
	if (method == DrawingProjectionMethod::FirstAngle)
	{
		// 俯上 / 正左下 / 右右下
		views.push_back(placeView(QStringLiteral("top"), QStringLiteral("俯视图"), QStringLiteral("top"), QPointF(0, 0),
								  topW, topH, topLVis, topLHid));
		views.push_back(placeView(QStringLiteral("front"), QStringLiteral("正视图"), QStringLiteral("front"),
								  QPointF(0, topH + gap), frontW, frontH, frontLVis, frontLHid));
		views.push_back(placeView(QStringLiteral("right"), QStringLiteral("右视图"), QStringLiteral("right"),
								  QPointF(col0W + gap, topH + gap), rightW, rightH, rightLVis, rightLHid));
	}
	else
	{
		// 第三角：正上 / 俯下 / 右右上
		views.push_back(placeView(QStringLiteral("front"), QStringLiteral("正视图"), QStringLiteral("front"),
								  QPointF(0, 0), frontW, frontH, frontLVis, frontLHid));
		views.push_back(placeView(QStringLiteral("top"), QStringLiteral("俯视图"), QStringLiteral("top"),
								  QPointF(0, frontH + gap), topW, topH, topLVis, topLHid));
		views.push_back(placeView(QStringLiteral("right"), QStringLiteral("右视图"), QStringLiteral("right"),
								  QPointF(qMax(frontW, topW) + gap, 0), rightW, rightH, rightLVis, rightLHid));
	}

	QRectF used;
	for (const auto& v : views)
		used = used.isValid() ? used.united(v.frame) : v.frame;
	double nextX = used.right() + gap;
	double nextY = used.top();
	if (hasIso)
	{
		views.push_back(placeView(QStringLiteral("iso"), QStringLiteral("轴测图"), QStringLiteral("iso"),
								  QPointF(nextX, nextY), isoW, isoH, isoLVis, isoLHid));
		nextY += isoH + gap;
	}
	if (hasSection)
	{
		views.push_back(placeView(QStringLiteral("section"), QStringLiteral("剖视图"), QStringLiteral("section"),
								  QPointF(nextX, nextY), secW, secH, secLVis, secLHid));
	}
	return views;
}
