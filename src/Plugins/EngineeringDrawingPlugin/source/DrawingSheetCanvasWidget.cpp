/// @file DrawingSheetCanvasWidget.cpp
/// @brief 工程图图幅：布局、拖拽、标注、局部放大、导出

#include "DrawingSheetCanvasWidget.h"

#include "DrawingExport.h"
#include "DrawingSidePanel.h"

#include <QDate>
#include <QDragEnterEvent>
#include <QHash>
#include <QInputDialog>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QKeyEvent>
#include <QLineEdit>
#include <QLineF>
#include <QMimeData>
#include <QMouseEvent>
#include <QPageLayout>
#include <QPageSize>
#include <QPainter>
#include <QPaintEvent>
#include <QPdfWriter>
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

/// Liang-Barsky：矩形裁剪线段
bool clipSegmentRect(QPointF a, QPointF b, const QRectF& r, QPointF& oa, QPointF& ob)
{
	const double xmin = r.left(), xmax = r.right(), ymin = r.top(), ymax = r.bottom();
	double u1 = 0.0, u2 = 1.0;
	const double dx = b.x() - a.x();
	const double dy = b.y() - a.y();
	auto clip = [&](double p, double q) {
		if (std::abs(p) < 1e-15)
			return q >= 0.0;
		const double t = q / p;
		if (p < 0.0)
		{
			if (t > u2)
				return false;
			if (t > u1)
				u1 = t;
		}
		else
		{
			if (t < u1)
				return false;
			if (t < u2)
				u2 = t;
		}
		return true;
	};
	if (!clip(-dx, a.x() - xmin) || !clip(dx, xmax - a.x()) || !clip(-dy, a.y() - ymin) || !clip(dy, ymax - a.y()))
		return false;
	oa = a + (b - a) * u1;
	ob = a + (b - a) * u2;
	return true;
}

QVector<DrawingSheetCanvasWidget::Polyline2d> clipScalePolylines(
	const QVector<DrawingSheetCanvasWidget::Polyline2d>& src, const QRectF& region, double scale)
{
	QVector<DrawingSheetCanvasWidget::Polyline2d> out;
	const QPointF tl = region.topLeft();
	auto mapPt = [&](const QPointF& p) { return (p - tl) * scale; };
	for (const auto& poly : src)
	{
		if (poly.points.size() < 2)
			continue;
		DrawingSheetCanvasWidget::Polyline2d cur;
		auto flush = [&]() {
			if (cur.points.size() >= 2)
				out.push_back(cur);
			cur.points.clear();
		};
		auto pushMapped = [&](const QPointF& p) {
			const QPointF m = mapPt(p);
			if (cur.points.isEmpty() || QLineF(cur.points.constLast(), m).length() > 1e-9)
				cur.points.push_back(m);
		};
		for (int i = 1; i < poly.points.size(); ++i)
		{
			QPointF a, b;
			if (!clipSegmentRect(poly.points[i - 1], poly.points[i], region, a, b))
			{
				flush();
				continue;
			}
			pushMapped(a);
			pushMapped(b);
		}
		flush();
	}
	return out;
}

bool fitCircle2d(const QVector<QPointF>& pts, QPointF& center, double& radius, double& maxErr)
{
	if (pts.size() < 3)
		return false;
	double sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, sumXY = 0, sumX3 = 0, sumY3 = 0, sumX2Y = 0, sumXY2 = 0;
	const int n = pts.size();
	for (const QPointF& p : pts)
	{
		const double x = p.x(), y = p.y();
		const double x2 = x * x, y2 = y * y;
		sumX += x;
		sumY += y;
		sumX2 += x2;
		sumY2 += y2;
		sumXY += x * y;
		sumX3 += x2 * x;
		sumY3 += y2 * y;
		sumX2Y += x2 * y;
		sumXY2 += x * y2;
	}
	const double C = n * sumX2 - sumX * sumX;
	const double D = n * sumXY - sumX * sumY;
	const double E = n * sumY2 - sumY * sumY;
	const double G = 0.5 * (n * sumX3 + n * sumXY2 - sumX * (sumX2 + sumY2));
	const double H = 0.5 * (n * sumY3 + n * sumX2Y - sumY * (sumX2 + sumY2));
	const double denom = C * E - D * D;
	if (std::abs(denom) < 1e-12)
		return false;
	const double cx = (G * E - D * H) / denom;
	const double cy = (C * H - D * G) / denom;
	center = QPointF(cx, cy);
	double sumR = 0;
	maxErr = 0;
	for (const QPointF& p : pts)
	{
		const double ri = QLineF(center, p).length();
		sumR += ri;
	}
	radius = sumR / n;
	if (radius < 1e-6)
		return false;
	for (const QPointF& p : pts)
		maxErr = qMax(maxErr, std::abs(QLineF(center, p).length() - radius));
	return maxErr <= qMax(0.05 * radius, 0.5);
}

} // namespace

DrawingSheetCanvasWidget::DrawingSheetCanvasWidget(QWidget* parent) : QWidget(parent)
{
	setMouseTracking(true);
	setFocusPolicy(Qt::StrongFocus);
	setMinimumSize(200, 200);
	setAttribute(Qt::WA_OpaquePaintEvent, true);
	setAcceptDrops(true);
	m_paper.date = QDate::currentDate().toString(Qt::ISODate);
	ensureDefaultLayer();
}

void DrawingSheetCanvasWidget::clearSheet()
{
	m_views.clear();
	m_dims.clear();
	m_notes.clear();
	m_sketch.clear();
	m_selectedSketchId = -1;
	m_selectedDimIndex = -1;
	m_selectedNoteIndex = -1;
	m_backendId.clear();
	m_needInitialFit = true;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setViews(const QVector<DrawingView>& views)
{
	m_views = views;
	for (DrawingView& v : m_views)
	{
		if (v.layerId.isEmpty())
			v.layerId = m_currentLayerId;
	}
	m_needInitialFit = true;
	applySheetScaleFromModel();
}

void DrawingSheetCanvasWidget::setDimensions(const QVector<SheetDimension>& dims)
{
	m_dims = dims;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setNotes(const QVector<SheetNote>& notes)
{
	m_notes = notes;
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setPaper(const SheetPaper& paper)
{
	m_paper = paper;
	emit sheetChanged();
	m_needInitialFit = true;
	update();
}

void DrawingSheetCanvasWidget::setBackendId(const QString& id)
{
	m_backendId = id;
	if (m_paper.title.isEmpty() && !id.isEmpty())
	{
		const int slash = qMax(id.lastIndexOf(QLatin1Char('/')), id.lastIndexOf(QLatin1Char('\\')));
		m_paper.title = (slash >= 0) ? id.mid(slash + 1) : id;
	}
}

QSizeF DrawingSheetCanvasWidget::paperSizeMm() const
{
	double w = 210.0, h = 297.0;
	switch (m_paper.size)
	{
	case DrawingPaperSize::A3:
		w = 297.0;
		h = 420.0;
		break;
	case DrawingPaperSize::A2:
		w = 420.0;
		h = 594.0;
		break;
	case DrawingPaperSize::A1:
		w = 594.0;
		h = 841.0;
		break;
	case DrawingPaperSize::A0:
		w = 841.0;
		h = 1189.0;
		break;
	case DrawingPaperSize::Custom:
		w = qMax(10.0, m_paper.customWidthMm);
		h = qMax(10.0, m_paper.customHeightMm);
		break;
	case DrawingPaperSize::A4:
	default:
		w = 210.0;
		h = 297.0;
		break;
	}
	if (m_paper.size != DrawingPaperSize::Custom && m_paper.landscape)
		qSwap(w, h);
	return QSizeF(w, h);
}

QRectF DrawingSheetCanvasWidget::paperRect() const
{
	const QSizeF s = paperSizeMm();
	return QRectF(0, 0, s.width(), s.height());
}

QRectF DrawingSheetCanvasWidget::paperDrawableRect() const
{
	const QRectF pr = paperRect();
	// 留边 + 右下标题栏占位
	const double m = 12.0;
	const double titleH = qMin(pr.height() * 0.18, 42.0);
	const double titleW = qMin(pr.width() * 0.42, 180.0);
	return pr.adjusted(m, m, -(m + 4.0), -(m + titleH * 0.35)).adjusted(0, 0, -qMax(0.0, titleW - pr.width() * 0.5), 0);
}

QRectF DrawingSheetCanvasWidget::viewsContentBounds() const
{
	QRectF box;
	for (const DrawingView& v : m_views)
		box = box.isValid() ? box.united(v.frame) : v.frame;
	for (const SheetDimension& d : m_dims)
	{
		box = box.united(QRectF(d.p1, QSizeF(0, 0)));
		box = box.united(QRectF(d.p2, QSizeF(0, 0)));
		if (d.kind == SheetDimension::Kind::Angle)
			box = box.united(QRectF(d.p3, QSizeF(0, 0)));
	}
	for (const SheetNote& n : m_notes)
	{
		box = box.united(QRectF(n.anchor, QSizeF(0, 0)));
		box = box.united(QRectF(n.textPos, QSizeF(0, 0)));
	}
	return box;
}

void DrawingSheetCanvasWidget::syncScaleTextFromSheetScale()
{
	const double s = m_paper.sheetScale;
	if (s <= 1e-12)
	{
		m_paper.scaleText = QStringLiteral("1:1");
		return;
	}
	if (std::abs(s - 1.0) < 1e-6)
		m_paper.scaleText = QStringLiteral("1:1");
	else if (s < 1.0)
	{
		const double n = 1.0 / s;
		m_paper.scaleText = (std::abs(n - std::round(n)) < 1e-3)
								? QStringLiteral("1:%1").arg(qRound(n))
								: QStringLiteral("1:%1").arg(n, 0, 'f', 2);
	}
	else
	{
		m_paper.scaleText = (std::abs(s - std::round(s)) < 1e-3)
								? QStringLiteral("%1:1").arg(qRound(s))
								: QStringLiteral("%1:1").arg(s, 0, 'f', 2);
	}
}

void DrawingSheetCanvasWidget::scaleSceneContent(double factor)
{
	if (!std::isfinite(factor) || std::abs(factor - 1.0) < 1e-12)
		return;
	auto scalePt = [&](QPointF& p) { p = QPointF(p.x() * factor, p.y() * factor); };
	for (DrawingView& v : m_views)
	{
		v.frame = QRectF(v.frame.x() * factor, v.frame.y() * factor, v.frame.width() * factor, v.frame.height() * factor);
		for (Polyline2d& poly : v.visible)
			for (QPointF& p : poly.points)
				scalePt(p);
		for (Polyline2d& poly : v.hidden)
			for (QPointF& p : poly.points)
				scalePt(p);
	}
	for (SheetDimension& d : m_dims)
	{
		scalePt(d.p1);
		scalePt(d.p2);
		scalePt(d.p3);
		d.textOffset = QPointF(d.textOffset.x() * factor, d.textOffset.y() * factor);
	}
	for (SheetNote& n : m_notes)
	{
		scalePt(n.anchor);
		scalePt(n.textPos);
	}
	for (SkPoint& pt : m_sketch.document().pointsMut())
	{
		pt.p.u *= factor;
		pt.p.v *= factor;
	}
	for (SkCircle& c : m_sketch.document().circlesMut())
		c.radius *= factor;
	for (SkEllipse& e : m_sketch.document().ellipsesMut())
	{
		e.majorR *= factor;
		e.minorR *= factor;
	}
}

void DrawingSheetCanvasWidget::placeViewsInPaper()
{
	if (!m_paper.visible || m_views.isEmpty())
		return;
	const QRectF drawable = paperDrawableRect();
	QRectF vb = viewsContentBounds();
	if (!vb.isValid() || vb.width() < 1e-6 || vb.height() < 1e-6)
		return;
	const QPointF delta = drawable.topLeft() - vb.topLeft() +
						  QPointF(qMax(0.0, (drawable.width() - vb.width()) * 0.5),
								  qMax(0.0, (drawable.height() - vb.height()) * 0.5));
	if (QLineF(QPointF(0, 0), delta).length() < 1e-9)
		return;
	for (DrawingView& v : m_views)
	{
		v.frame.translate(delta);
		for (Polyline2d& poly : v.visible)
			for (QPointF& p : poly.points)
				p += delta;
		for (Polyline2d& poly : v.hidden)
			for (QPointF& p : poly.points)
				p += delta;
	}
	for (SheetDimension& d : m_dims)
	{
		d.p1 += delta;
		d.p2 += delta;
		d.p3 += delta;
	}
	for (SheetNote& n : m_notes)
	{
		n.anchor += delta;
		n.textPos += delta;
	}
	for (SkPoint& pt : m_sketch.document().pointsMut())
	{
		pt.p.u += delta.x();
		pt.p.v += delta.y();
	}
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::setSheetScale(double scale, bool rescaleContent)
{
	if (!std::isfinite(scale) || scale < 1e-6)
		return;
	const double old = m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0;
	if (rescaleContent && !m_views.isEmpty())
		scaleSceneContent(scale / old);
	m_paper.sheetScale = scale;
	syncScaleTextFromSheetScale();
	if (rescaleContent && !m_views.isEmpty())
		placeViewsInPaper();
	emit sheetChanged();
	update();
}

void DrawingSheetCanvasWidget::applySheetScaleFromModel()
{
	const double s = m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0;
	syncScaleTextFromSheetScale();
	// 仅缩新生成的视图；标注/草图可能已是图面坐标，不能再乘 sheetScale
	if (std::abs(s - 1.0) > 1e-12)
	{
		auto scalePt = [&](QPointF& p) { p = QPointF(p.x() * s, p.y() * s); };
		for (DrawingView& v : m_views)
		{
			v.frame = QRectF(v.frame.x() * s, v.frame.y() * s, v.frame.width() * s, v.frame.height() * s);
			for (Polyline2d& poly : v.visible)
				for (QPointF& p : poly.points)
					scalePt(p);
			for (Polyline2d& poly : v.hidden)
				for (QPointF& p : poly.points)
					scalePt(p);
		}
	}
	placeViewsInPaper();
	m_needInitialFit = true;
	fitToView();
	emit sheetChanged();
	update();
}

bool DrawingSheetCanvasWidget::fitViewsToPaper()
{
	if (!m_paper.visible || m_views.isEmpty())
		return false;
	const QRectF drawable = paperDrawableRect();
	QRectF vb = viewsContentBounds();
	if (!vb.isValid() || vb.width() < 1e-6 || vb.height() < 1e-6)
		return false;
	const double sx = drawable.width() / vb.width();
	const double sy = drawable.height() / vb.height();
	const double k = qMin(sx, sy) * 0.92;
	if (!std::isfinite(k) || k < 1e-6)
		return false;
	scaleSceneContent(k);
	m_paper.sheetScale = (m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0) * k;
	syncScaleTextFromSheetScale();
	placeViewsInPaper();
	m_needInitialFit = true;
	fitToView();
	emit sheetChanged();
	emit statusMessage(QStringLiteral("已适应图幅，比例 %1").arg(m_paper.scaleText));
	update();
	return true;
}

bool DrawingSheetCanvasWidget::isEmpty() const
{
	return m_views.isEmpty() && m_dims.isEmpty() && m_notes.isEmpty() && m_sketch.document().lines().empty() &&
		   m_sketch.document().arcs().empty() && m_sketch.document().circles().empty() &&
		   m_sketch.document().splines().empty();
}

void DrawingSheetCanvasWidget::ensureDefaultLayer()
{
	if (!m_layers.isEmpty())
		return;
	SheetLayer def;
	def.id = defaultLayerId();
	def.name = QStringLiteral("0");
	def.visible = true;
	def.locked = false;
	m_layers.push_back(def);
	m_currentLayerId = def.id;
	m_nextLayerSeq = 1;
}

int DrawingSheetCanvasWidget::layerIndex(const QString& layerId) const
{
	for (int i = 0; i < m_layers.size(); ++i)
	{
		if (m_layers[i].id == layerId)
			return i;
	}
	return -1;
}

const DrawingSheetCanvasWidget::SheetLayer* DrawingSheetCanvasWidget::layerById(const QString& layerId) const
{
	const int idx = layerIndex(layerId.isEmpty() ? defaultLayerId() : layerId);
	return idx >= 0 ? &m_layers[idx] : nullptr;
}

bool DrawingSheetCanvasWidget::isLayerDrawable(const QString& layerId) const
{
	const SheetLayer* L = layerById(layerId);
	return L && L->visible;
}

bool DrawingSheetCanvasWidget::isLayerEditable(const QString& layerId) const
{
	const SheetLayer* L = layerById(layerId);
	return L && L->visible && !L->locked;
}

QString DrawingSheetCanvasWidget::uniqueLayerName(const QString& base) const
{
	QString name = base.trimmed().isEmpty() ? QStringLiteral("图层") : base.trimmed();
	auto exists = [&](const QString& n) {
		for (const SheetLayer& L : m_layers)
		{
			if (L.name.compare(n, Qt::CaseInsensitive) == 0)
				return true;
		}
		return false;
	};
	if (!exists(name))
		return name;
	for (int i = 1; i < 1000; ++i)
	{
		const QString cand = QStringLiteral("%1_%2").arg(name).arg(i);
		if (!exists(cand))
			return cand;
	}
	return name + QStringLiteral("_x");
}

bool DrawingSheetCanvasWidget::setCurrentLayer(const QString& layerId)
{
	if (layerIndex(layerId) < 0)
		return false;
	if (m_currentLayerId == layerId)
		return true;
	m_currentLayerId = layerId;
	emit layersChanged();
	return true;
}

QString DrawingSheetCanvasWidget::addLayer(const QString& name)
{
	ensureDefaultLayer();
	SheetLayer L;
	L.id = QStringLiteral("L%1").arg(m_nextLayerSeq++);
	L.name = uniqueLayerName(name);
	m_layers.push_back(L);
	m_currentLayerId = L.id;
	emit layersChanged();
	emit sheetChanged();
	return L.id;
}

bool DrawingSheetCanvasWidget::renameLayer(const QString& layerId, const QString& name)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	const QString n = name.trimmed();
	if (n.isEmpty())
		return false;
	for (int i = 0; i < m_layers.size(); ++i)
	{
		if (i != idx && m_layers[i].name.compare(n, Qt::CaseInsensitive) == 0)
			return false;
	}
	m_layers[idx].name = n;
	emit layersChanged();
	emit sheetChanged();
	return true;
}

void DrawingSheetCanvasWidget::migrateLayerEntities(const QString& fromId, const QString& toId)
{
	for (DrawingView& v : m_views)
	{
		if (v.layerId == fromId)
			v.layerId = toId;
	}
	for (SheetDimension& d : m_dims)
	{
		if (d.layerId == fromId)
			d.layerId = toId;
	}
	for (SheetNote& n : m_notes)
	{
		if (n.layerId == fromId)
			n.layerId = toId;
	}
	m_sketch.remapLayer(fromId, toId);
}

bool DrawingSheetCanvasWidget::removeLayer(const QString& layerId)
{
	if (layerId == defaultLayerId())
		return false;
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	migrateLayerEntities(layerId, defaultLayerId());
	m_layers.removeAt(idx);
	if (m_currentLayerId == layerId)
		m_currentLayerId = defaultLayerId();
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::setLayerVisible(const QString& layerId, bool visible)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	if (m_layers[idx].visible == visible)
		return true;
	m_layers[idx].visible = visible;
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::setLayerLocked(const QString& layerId, bool locked)
{
	const int idx = layerIndex(layerId);
	if (idx < 0)
		return false;
	if (m_layers[idx].locked == locked)
		return true;
	m_layers[idx].locked = locked;
	emit layersChanged();
	emit sheetChanged();
	update();
	return true;
}

bool DrawingSheetCanvasWidget::reassignSelectionToCurrentLayer()
{
	if (!isLayerEditable(m_currentLayerId))
	{
		emit statusMessage(QStringLiteral("当前层不可编辑"));
		return false;
	}
	bool changed = false;
	if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
	{
		m_dims[m_selectedDimIndex].layerId = m_currentLayerId;
		changed = true;
	}
	else if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
	{
		m_notes[m_selectedNoteIndex].layerId = m_currentLayerId;
		changed = true;
	}
	else if (m_selectedSketchId >= 0)
	{
		m_sketch.setLayerOf(m_selectedSketchId, m_currentLayerId);
		changed = true;
	}
	if (!changed)
	{
		emit statusMessage(QStringLiteral("请先选中尺寸/文字/草图"));
		return false;
	}
	emit sheetChanged();
	emit statusMessage(QStringLiteral("已移到当前层"));
	update();
	return true;
}

int DrawingSheetCanvasWidget::hitSketchEntity(const QPointF& scenePos, bool requireEditable) const
{
	return m_sketch.hitTestEntity(scenePos, snapTolMm(), [&](int id) {
		const QString lid = m_sketch.layerOf(id);
		if (requireEditable)
			return isLayerEditable(lid);
		return isLayerDrawable(lid);
	});
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
			const int n = poly.points.size();
			if (n == 0)
				continue;
			for (int i = 0; i < n; ++i)
				pts.push_back(poly.points[i]);
			// 每段中点，便于点轮廓标尺寸
			for (int i = 1; i < n; ++i)
				pts.push_back((poly.points[i - 1] + poly.points[i]) * 0.5);
		}
	};
	for (const DrawingView& v : m_views)
	{
		if (!isLayerDrawable(v.layerId))
			continue;
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
	m_notePicking = false;
	m_detailDragging = false;
	m_selectedSketchId = -1;
	m_selectedDimIndex = -1;
	m_selectedNoteIndex = -1;
	syncSketchTool();
	QString tip = QStringLiteral("选择/拖视图");
	switch (tool)
	{
	case DrawingCanvasTool::LinearDim:
		tip = QStringLiteral("线性尺寸：点视图轮廓或草图点，再点放置尺寸线");
		break;
	case DrawingCanvasTool::DimRadius:
		tip = QStringLiteral("半径：点圆/弧，再点文字位置");
		break;
	case DrawingCanvasTool::DimDiameter:
		tip = QStringLiteral("直径：点圆/弧，再点文字位置");
		break;
	case DrawingCanvasTool::DimAngle:
		tip = QStringLiteral("角度：顶点 → 第一边上点 → 第二边上点");
		break;
	case DrawingCanvasTool::NoteLeader:
		tip = QStringLiteral("文字：点锚点，再点文字位置");
		break;
	case DrawingCanvasTool::DetailRegion:
		tip = QStringLiteral("局部放大：拖框选区域（倍率见工具栏）");
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

void DrawingSheetCanvasWidget::setDetailScale(double scale)
{
	m_detailScale = qBound(1.5, scale, 10.0);
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
	if (m_paper.visible)
		box = paperRect();
	for (const DrawingView& v : m_views)
		box = box.isValid() ? box.united(v.frame) : v.frame;
	for (const SheetDimension& d : m_dims)
	{
		box = box.united(QRectF(d.p1, QSizeF(0, 0)));
		box = box.united(QRectF(d.p2, QSizeF(0, 0)));
		if (d.kind == SheetDimension::Kind::Angle)
			box = box.united(QRectF(d.p3, QSizeF(0, 0)));
	}
	for (const SheetNote& n : m_notes)
	{
		box = box.united(QRectF(n.anchor, QSizeF(0, 0)));
		box = box.united(QRectF(n.textPos, QSizeF(0, 0)));
	}
	for (const SheetSketchPolyline& poly : m_sketch.tessellate())
	{
		for (const QPointF& pt : poly.points)
			box = box.isValid() ? box.united(QRectF(pt, QSizeF(0, 0))) : QRectF(pt, QSizeF(1, 1));
	}
	if (!box.isValid())
		return paperRect().isValid() ? paperRect() : QRectF(0, 0, 200, 200);
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
	return drawing_export::writeSvg(filePath, m_views, m_dims, m_notes, m_sketch.tessellate(), m_paper, m_projection,
									m_layers, m_sketch.entityLayers());
}

bool DrawingSheetCanvasWidget::exportDxf(const QString& filePath) const
{
	return drawing_export::writeDxf(filePath, m_views, m_dims, m_notes, m_sketch.tessellate(), m_paper, m_projection,
									m_layers, m_sketch.entityLayers());
}

bool DrawingSheetCanvasWidget::exportPdf(const QString& filePath)
{
	if (filePath.isEmpty())
		return false;

	const QRectF sceneBox = m_paper.visible ? paperRect() : contentBounds();
	if (!sceneBox.isValid() || sceneBox.width() < 1e-3 || sceneBox.height() < 1e-3)
		return false;

	QPdfWriter writer(filePath);
	writer.setTitle(m_paper.title.isEmpty() ? QStringLiteral("Engineering Drawing") : m_paper.title);
	writer.setCreator(QStringLiteral("CloudSim EngineeringDrawing"));
	const QSizeF mm = paperSizeMm();
	writer.setPageSizeMM(mm);
	writer.setResolution(72);

	QPainter p(&writer);
	if (!p.isActive())
		return false;
	p.setRenderHint(QPainter::Antialiasing, true);

	const QRectF pageRect(0, 0, writer.width(), writer.height());
	p.fillRect(pageRect, Qt::white);

	// 场景 mm → PDF 页，居中铺满（与屏幕 zoom/pan 解耦）
	const double zx = pageRect.width() / sceneBox.width();
	const double zy = pageRect.height() / sceneBox.height();
	const double z = qMin(zx, zy);
	const double oldZoom = m_zoom;
	const QPointF oldPan = m_panOffset;
	m_zoom = z;
	m_panOffset = QPointF(pageRect.left() - sceneBox.left() * z + (pageRect.width() - sceneBox.width() * z) * 0.5,
						  pageRect.top() - sceneBox.top() * z + (pageRect.height() - sceneBox.height() * z) * 0.5);
	paintSheet(p, true);
	m_zoom = oldZoom;
	m_panOffset = oldPan;
	p.end();
	return true;
}

void DrawingSheetCanvasWidget::paintSheet(QPainter& p, bool forExport) const
{
	if (!forExport)
		drawGrid(p);
	drawPaper(p);
	for (const DrawingView& v : m_views)
	{
		if (!isLayerDrawable(v.layerId))
			continue;
		drawView(p, v);
	}
	drawSketch(p, !forExport);
	for (const SheetDimension& d : m_dims)
	{
		if (!isLayerDrawable(d.layerId))
			continue;
		drawDimension(p, d);
	}
	for (const SheetNote& n : m_notes)
	{
		if (!isLayerDrawable(n.layerId))
			continue;
		drawNote(p, n);
	}
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

void DrawingSheetCanvasWidget::drawPaper(QPainter& p) const
{
	if (!m_paper.visible)
		return;
	const QRectF pr = paperRect();
	const QRectF outer = QRectF(sceneToWidget(pr.topLeft()), sceneToWidget(pr.bottomRight())).normalized();
	p.save();
	p.setPen(QPen(QColor(40, 45, 55), 2.0));
	p.setBrush(Qt::NoBrush);
	p.drawRect(outer);
	const QRectF inner = outer.adjusted(8 * m_zoom, 8 * m_zoom, -8 * m_zoom, -8 * m_zoom);
	p.setPen(QPen(QColor(60, 70, 85), 1.0));
	p.drawRect(inner);

	// 右下角标题栏
	const double tw = qMin(outer.width() * 0.42, 180.0 * m_zoom);
	const double th = qMin(outer.height() * 0.18, 42.0 * m_zoom);
	const QRectF block(inner.right() - tw, inner.bottom() - th, tw, th);
	p.setPen(QPen(QColor(40, 45, 55), 1.2));
	p.drawRect(block);
	p.drawLine(block.left(), block.center().y(), block.right(), block.center().y());
	p.drawLine(block.left() + tw * 0.55, block.top(), block.left() + tw * 0.55, block.bottom());

	const QString proj = m_projection == DrawingProjectionMethod::ThirdAngle ? QStringLiteral("第三角")
																			: QStringLiteral("第一角");
	p.setPen(QColor(30, 35, 45));
	QFont f = p.font();
	f.setPointSizeF(qMax(7.0, 8.0 * qMin(1.5, m_zoom)));
	p.setFont(f);
	const QString title = m_paper.title.isEmpty() ? QStringLiteral("未命名") : m_paper.title;
	p.drawText(block.adjusted(4, 2, -4, -th * 0.5), Qt::AlignLeft | Qt::AlignVCenter, title);
	p.drawText(QRectF(block.left() + 4, block.center().y(), tw * 0.55 - 8, th * 0.5), Qt::AlignLeft | Qt::AlignVCenter,
			   QStringLiteral("比例 %1").arg(m_paper.scaleText));
	p.drawText(QRectF(block.left() + tw * 0.55 + 4, block.top() + 2, tw * 0.45 - 8, th * 0.5 - 2),
			   Qt::AlignLeft | Qt::AlignVCenter, proj);
	p.drawText(QRectF(block.left() + tw * 0.55 + 4, block.center().y(), tw * 0.45 - 8, th * 0.5),
			   Qt::AlignLeft | Qt::AlignVCenter,
			   QStringLiteral("%1  mm").arg(m_paper.date.isEmpty() ? QStringLiteral("-") : m_paper.date));
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

void DrawingSheetCanvasWidget::drawSketch(QPainter& p, bool interactive) const
{
	p.save();
	for (const SheetSketchPolyline& poly : m_sketch.tessellate())
	{
		if (poly.points.size() < 2)
			continue;
		if (!isLayerDrawable(m_sketch.layerOf(poly.entityId)))
			continue;
		const bool sel = interactive && poly.entityId >= 0 && poly.entityId == m_selectedSketchId;
		QPen pen(poly.construction ? QColor(140, 145, 155) : QColor(30, 120, 180), sel ? 2.4 : 1.6,
				 poly.construction ? Qt::DashLine : Qt::SolidLine);
		p.setPen(pen);
		QPolygonF w;
		for (const QPointF& pt : poly.points)
			w << sceneToWidget(pt);
		p.drawPolyline(w);
	}
	if (interactive)
	{
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
	if (dim.kind == SheetDimension::Kind::Angle)
	{
		const QPointF a = dim.p2 - dim.p1;
		const QPointF b = dim.p3 - dim.p1;
		const double la = std::hypot(a.x(), a.y());
		const double lb = std::hypot(b.x(), b.y());
		if (la < 1e-9 || lb < 1e-9)
			return 0.0;
		double c = (a.x() * b.x() + a.y() * b.y()) / (la * lb);
		c = qBound(-1.0, c, 1.0);
		return std::acos(c) * 180.0 / 3.141592653589793;
	}
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
	if (dim.kind == SheetDimension::Kind::Angle)
		return QStringLiteral("%1°").arg(v, 0, 'f', 1);
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
	else if (dim.kind == SheetDimension::Kind::Angle)
	{
		const QPointF v = dim.p1;
		p.drawLine(sceneToWidget(v), sceneToWidget(dim.p2));
		p.drawLine(sceneToWidget(v), sceneToWidget(dim.p3));
		const double a0 = std::atan2(dim.p2.y() - v.y(), dim.p2.x() - v.x());
		const double a1 = std::atan2(dim.p3.y() - v.y(), dim.p3.x() - v.x());
		double span = a1 - a0;
		while (span <= -3.141592653589793)
			span += 2.0 * 3.141592653589793;
		while (span > 3.141592653589793)
			span -= 2.0 * 3.141592653589793;
		const double rad = qMax(12.0, 0.35 * qMin(QLineF(v, dim.p2).length(), QLineF(v, dim.p3).length()));
		QRectF arcBox(sceneToWidget(QPointF(v.x() - rad, v.y() - rad)),
					  sceneToWidget(QPointF(v.x() + rad, v.y() + rad)));
		arcBox = arcBox.normalized();
		const int start16 = static_cast<int>(-a0 * 180.0 / 3.141592653589793 * 16.0);
		const int span16 = static_cast<int>(-span * 180.0 / 3.141592653589793 * 16.0);
		p.drawArc(arcBox, start16, span16);
		const double amid = a0 + span * 0.5;
		const QPointF textScene = v + QPointF(std::cos(amid), std::sin(amid)) * (rad + 8.0) + dim.textOffset;
		p.drawText(sceneToWidget(textScene), dimensionText(dim));
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

void DrawingSheetCanvasWidget::drawNote(QPainter& p, const SheetNote& note) const
{
	p.save();
	const bool sel = m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size() &&
					 m_notes[m_selectedNoteIndex].id == note.id;
	p.setPen(QPen(sel ? QColor(142, 68, 173) : QColor(52, 73, 94), sel ? 1.6 : 1.1));
	p.drawLine(sceneToWidget(note.anchor), sceneToWidget(note.textPos));
	p.drawEllipse(sceneToWidget(note.anchor), 3, 3);
	p.drawText(sceneToWidget(note.textPos) + QPointF(4, -2), note.text);
	p.restore();
}

void DrawingSheetCanvasWidget::paintEvent(QPaintEvent*)
{
	// 与工艺流程一致：首次有内容时在 paint 里补 fit，避免仅靠 resize
	if (m_needInitialFit && (!isEmpty() || m_paper.visible) && width() >= 10 && height() >= 10)
		fitToView();

	QPainter p(this);
	p.fillRect(rect(), QColor(0xF5, 0xF7, 0xFA));
	p.setRenderHint(QPainter::Antialiasing, true);
	paintSheet(p, false);
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
		p.drawLine(sceneToWidget(m_dimP1), sceneToWidget(m_dimP2));
	}
	if (m_dimPicking && m_tool == DrawingCanvasTool::DimAngle && m_dimPickStep >= 1)
	{
		p.setPen(QPen(QColor(192, 57, 43), 1.0, Qt::DashLine));
		p.drawLine(sceneToWidget(m_dimP1), sceneToWidget(m_dimP2));
		if (m_dimPickStep >= 2)
			p.drawLine(sceneToWidget(m_dimP1), sceneToWidget(m_dimP3));
	}
	if (m_notePicking)
	{
		p.setPen(QPen(QColor(52, 73, 94), 1.0, Qt::DashLine));
		p.drawLine(sceneToWidget(m_noteAnchor), sceneToWidget(m_dimP2));
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
		if (!isLayerEditable(m_views[i].layerId))
			continue;
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
	if (!isLayerEditable(v.layerId))
		return;
	v.frame.translate(deltaScene);
	for (Polyline2d& poly : v.visible)
		for (QPointF& pt : poly.points)
			pt += deltaScene;
	for (Polyline2d& poly : v.hidden)
		for (QPointF& pt : poly.points)
			pt += deltaScene;
	for (SheetDimension& d : m_dims)
	{
		if (d.anchorViewId != v.id)
			continue;
		d.p1 += deltaScene;
		d.p2 += deltaScene;
		if (d.kind == SheetDimension::Kind::Angle)
			d.p3 += deltaScene;
	}
	for (SheetNote& n : m_notes)
	{
		if (n.anchorViewId != v.id)
			continue;
		n.anchor += deltaScene;
		n.textPos += deltaScene;
	}
	emit sheetChanged();
	update();
}

QString DrawingSheetCanvasWidget::inferAnchorViewId(const QPointF& scenePos) const
{
	const int idx = hitViewIndex(scenePos);
	return idx >= 0 ? m_views[idx].id : QString();
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
	const double useScale = scale > 1e-6 ? scale : m_detailScale;
	const DrawingView& parent = m_views[parentIdx];
	const QRectF region = regionScene.normalized();
	if (region.width() < 1e-3 || region.height() < 1e-3)
		return false;

	auto clipScale = [&](const QVector<Polyline2d>& src) { return clipScalePolylines(src, region, useScale); };

	DrawingView detail;
	detail.id = QStringLiteral("detail_%1").arg(m_nextDetailId++);
	detail.title = QStringLiteral("局部 ×%1").arg(useScale, 0, 'f', 1);
	detail.kind = QStringLiteral("detail");
	detail.parentViewId = parentViewId;
	detail.contentScale = useScale;
	detail.visible = clipScale(parent.visible);
	detail.hidden = clipScale(parent.hidden);
	const QRectF localBox = boundsOfPolylines(detail.visible, detail.hidden);
	detail.visible = offsetPolylines(detail.visible, -localBox.topLeft());
	detail.hidden = offsetPolylines(detail.hidden, -localBox.topLeft());
	const QPointF origin(contentBounds().right() + 40.0, contentBounds().top());
	detail.visible = offsetPolylines(detail.visible, origin);
	detail.hidden = offsetPolylines(detail.hidden, origin);
	detail.frame = QRectF(origin, localBox.size()).adjusted(-8, -18, 8, 8);
	detail.layerId = m_currentLayerId;
	m_views.push_back(detail);
	emit sheetChanged();
	fitToView();
	return true;
}

bool DrawingSheetCanvasWidget::removeView(const QString& viewId)
{
	for (int i = 0; i < m_views.size(); ++i)
	{
		if (m_views[i].id != viewId)
			continue;
		if (!isLayerEditable(m_views[i].layerId))
		{
			emit statusMessage(QStringLiteral("图层已锁定"));
			return false;
		}
		m_views.removeAt(i);
		emit sheetChanged();
		update();
		return true;
	}
	return false;
}

bool DrawingSheetCanvasWidget::renameView(const QString& viewId, const QString& title)
{
	const QString t = title.trimmed();
	if (t.isEmpty())
		return false;
	for (DrawingView& v : m_views)
	{
		if (v.id != viewId)
			continue;
		v.title = t;
		emit sheetChanged();
		update();
		return true;
	}
	return false;
}

bool DrawingSheetCanvasWidget::setDetailViewScale(const QString& viewId, double scale)
{
	if (scale < 1.5 || scale > 10.0)
		return false;
	for (DrawingView& v : m_views)
	{
		if (v.id != viewId || v.kind != QLatin1String("detail"))
			continue;
		if (!isLayerEditable(v.layerId))
		{
			emit statusMessage(QStringLiteral("图层已锁定"));
			return false;
		}
		const double old = v.contentScale > 1e-9 ? v.contentScale : 1.0;
		const double ratio = scale / old;
		const QPointF origin = v.frame.topLeft() + QPointF(8, 18);
		v.visible = scalePolylinesAbout(v.visible, origin, ratio);
		v.hidden = scalePolylinesAbout(v.hidden, origin, ratio);
		v.contentScale = scale;
		v.title = QStringLiteral("局部 ×%1").arg(scale, 0, 'f', 1);
		const QRectF localBox = boundsOfPolylines(v.visible, v.hidden);
		v.frame = localBox.adjusted(-8, -18, 8, 8);
		emit sheetChanged();
		update();
		return true;
	}
	return false;
}

QVector<DrawingSheetCanvasWidget::DrawingView> DrawingSheetCanvasWidget::detailViews() const
{
	QVector<DrawingView> out;
	for (const DrawingView& v : m_views)
	{
		if (v.kind == QLatin1String("detail"))
			out.push_back(v);
	}
	return out;
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

bool DrawingSheetCanvasWidget::resolveHlrCircleNear(const QPointF& scenePos, QPointF& center, QPointF& rim,
												   double& radius) const
{
	const double tol = snapTolMm() * 3.0;
	double bestDist = 1e100;
	QVector<QPointF> bestPts;
	for (const DrawingView& v : m_views)
	{
		if (!isLayerDrawable(v.layerId))
			continue;
		auto consider = [&](const QVector<Polyline2d>& polys) {
			for (const Polyline2d& poly : polys)
			{
				if (poly.points.size() < 3)
					continue;
				double minD = 1e100;
				for (const QPointF& p : poly.points)
					minD = qMin(minD, QLineF(p, scenePos).length());
				for (int i = 1; i < poly.points.size(); ++i)
				{
					QLineF seg(poly.points[i - 1], poly.points[i]);
					const QPointF ab = seg.p2() - seg.p1();
					const double len2 = QPointF::dotProduct(ab, ab);
					double t = len2 > 1e-12 ? QPointF::dotProduct(scenePos - seg.p1(), ab) / len2 : 0.0;
					t = qBound(0.0, t, 1.0);
					minD = qMin(minD, QLineF(seg.p1() + ab * t, scenePos).length());
				}
				if (minD > tol || minD >= bestDist)
					continue;
				QPointF c;
				double r = 0, err = 0;
				if (!fitCircle2d(poly.points, c, r, err))
					continue;
				bestDist = minD;
				bestPts = poly.points;
				center = c;
				radius = r;
			}
		};
		consider(v.visible);
		consider(v.hidden);
	}
	if (bestPts.isEmpty())
		return false;
	QLineF ray(center, scenePos);
	if (ray.length() < 1e-6)
		ray = QLineF(center, bestPts.constFirst());
	ray.setLength(radius);
	rim = ray.p2();
	return true;
}

int DrawingSheetCanvasWidget::hitDimensionIndex(const QPointF& scenePos) const
{
	const double tol = snapTolMm();
	for (int i = m_dims.size() - 1; i >= 0; --i)
	{
		const SheetDimension& d = m_dims[i];
		if (!isLayerEditable(d.layerId))
			continue;
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

int DrawingSheetCanvasWidget::hitNoteIndex(const QPointF& scenePos) const
{
	const double tol = snapTolMm() * 2.0;
	for (int i = m_notes.size() - 1; i >= 0; --i)
	{
		const SheetNote& n = m_notes[i];
		if (!isLayerEditable(n.layerId))
			continue;
		if (QLineF(n.textPos, scenePos).length() <= tol || QLineF(n.anchor, scenePos).length() <= tol)
			return i;
		QLineF ray(n.anchor, n.textPos);
		const QPointF ab = ray.p2() - ray.p1();
		const double len2 = QPointF::dotProduct(ab, ab);
		double t = len2 > 1e-12 ? QPointF::dotProduct(scenePos - ray.p1(), ab) / len2 : 0.0;
		t = qBound(0.0, t, 1.0);
		if (QLineF(ray.p1() + ab * t, scenePos).length() <= tol)
			return i;
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
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		m_sketch.press(scenePos, true, snapTolMm(), extraSnap, m_currentLayerId);
		emit sheetChanged();
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && isSketchTool(m_tool))
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		m_sketch.press(scenePos, false, snapTolMm(), extraSnap, m_currentLayerId);
		emit sheetChanged();
		update();
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::LinearDim)
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
		const QPointF sp = SheetSketchAdapter::toScene(snapped);
		if (!m_dimPicking)
		{
			m_dimP1 = sp;
			m_dimP2 = sp;
			m_dimPicking = true;
			m_dimPickStep = 1;
			emit statusMessage(QStringLiteral("尺寸：再点第二点（可吸附轮廓）"));
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
			dim.anchorViewId = inferAnchorViewId((m_dimP1 + m_dimP2) * 0.5);
			dim.layerId = m_currentLayerId;
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
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		if (!m_dimPicking)
		{
			QPointF c, rim;
			double r = 0;
			const int ent = hitSketchEntity(scenePos, false);
			bool ok = (ent >= 0 && resolveCircleDim(ent, c, rim, r));
			if (ok)
				m_dimEntityId = ent;
			else
			{
				m_dimEntityId = -1;
				ok = resolveHlrCircleNear(scenePos, c, rim, r);
			}
			if (ok)
			{
				m_dimP1 = c;
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
				const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
				m_dimP1 = SheetSketchAdapter::toScene(snapped);
				m_dimP2 = m_dimP1;
				m_dimPicking = true;
				m_dimPickStep = 10;
				m_dimEntityId = -1;
				emit statusMessage(QStringLiteral("未拟合到圆：再点圆周一点"));
			}
		}
		else if (m_dimPickStep == 10)
		{
			const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
			m_dimP2 = SheetSketchAdapter::toScene(snapped);
			if (QLineF(m_dimP1, m_dimP2).length() < 1e-6)
			{
				emit statusMessage(QStringLiteral("半径过小，请重选圆周点"));
				event->accept();
				return;
			}
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
			dim.anchorViewId = inferAnchorViewId(m_dimP1);
			dim.layerId = m_currentLayerId;
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

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::DimAngle)
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
		const QPointF sp = SheetSketchAdapter::toScene(snapped);
		if (!m_dimPicking)
		{
			m_dimP1 = m_dimP2 = m_dimP3 = sp;
			m_dimPicking = true;
			m_dimPickStep = 1;
			emit statusMessage(QStringLiteral("角度：再点第一边上一点"));
		}
		else if (m_dimPickStep == 1)
		{
			m_dimP2 = sp;
			m_dimPickStep = 2;
			emit statusMessage(QStringLiteral("角度：再点第二边上一点"));
		}
		else
		{
			SheetDimension dim;
			dim.kind = SheetDimension::Kind::Angle;
			dim.id = QStringLiteral("dim_%1").arg(m_nextDimId++);
			dim.p1 = m_dimP1;
			dim.p2 = m_dimP2;
			dim.p3 = sp;
			dim.anchorViewId = inferAnchorViewId(m_dimP1);
			dim.layerId = m_currentLayerId;
			m_dims.push_back(dim);
			m_dimPicking = false;
			m_dimPickStep = 0;
			emit sheetChanged();
			emit statusMessage(QStringLiteral("已添加角度尺寸"));
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::NoteLeader)
	{
		if (!isLayerEditable(m_currentLayerId))
		{
			emit statusMessage(QStringLiteral("当前层已锁定或隐藏"));
			event->accept();
			return;
		}
		const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), extraSnap, nullptr);
		const QPointF sp = SheetSketchAdapter::toScene(snapped);
		if (!m_notePicking)
		{
			m_noteAnchor = sp;
			m_dimP2 = sp;
			m_notePicking = true;
			emit statusMessage(QStringLiteral("文字：再点文字位置"));
		}
		else
		{
			bool ok = false;
			const QString text = QInputDialog::getText(this, QStringLiteral("引线文字"), QStringLiteral("内容"),
													  QLineEdit::Normal, QStringLiteral("注"), &ok);
			if (ok && !text.trimmed().isEmpty())
			{
				SheetNote note;
				note.id = QStringLiteral("note_%1").arg(m_nextNoteId++);
				note.anchor = m_noteAnchor;
				note.textPos = sp;
				note.text = text.trimmed();
				note.anchorViewId = inferAnchorViewId(m_noteAnchor);
				note.layerId = m_currentLayerId;
				m_notes.push_back(note);
				emit sheetChanged();
				emit statusMessage(QStringLiteral("已添加文字"));
			}
			m_notePicking = false;
			update();
		}
		event->accept();
		return;
	}

	if (event->button() == Qt::LeftButton && m_tool == DrawingCanvasTool::SelectEntity)
	{
		m_selectedDimIndex = hitDimensionIndex(scenePos);
		m_selectedNoteIndex = -1;
		m_selectedSketchId = -1;
		if (m_selectedDimIndex < 0)
			m_selectedNoteIndex = hitNoteIndex(scenePos);
		if (m_selectedDimIndex < 0 && m_selectedNoteIndex < 0)
			m_selectedSketchId = hitSketchEntity(scenePos, true);
		emit statusMessage(m_selectedDimIndex >= 0 || m_selectedNoteIndex >= 0 || m_selectedSketchId >= 0
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
	if (m_dimPicking && m_tool == DrawingCanvasTool::DimAngle)
	{
		const SkVec2 snapped = m_sketch.snapScene(scenePos, snapTolMm(), collectViewSnapPoints(), nullptr);
		const QPointF sp = SheetSketchAdapter::toScene(snapped);
		if (m_dimPickStep == 1)
			m_dimP2 = sp;
		else
			m_dimP3 = sp;
		update();
		event->accept();
		return;
	}
	if (m_notePicking)
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
			addDetailView(m_views[parentIdx].id, region, m_detailScale);
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
	if (!isLayerEditable(m_dims[idx].layerId))
	{
		emit statusMessage(QStringLiteral("图层已锁定"));
		event->accept();
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
		m_notePicking = false;
		m_selectedSketchId = -1;
		m_selectedDimIndex = -1;
		m_selectedNoteIndex = -1;
		update();
		event->accept();
		return;
	}
	if (event->key() == Qt::Key_Delete || event->key() == Qt::Key_Backspace)
	{
		bool changed = false;
		if (m_selectedDimIndex >= 0 && m_selectedDimIndex < m_dims.size())
		{
			if (!isLayerEditable(m_dims[m_selectedDimIndex].layerId))
			{
				emit statusMessage(QStringLiteral("图层已锁定，无法删除"));
				event->accept();
				return;
			}
			m_dims.removeAt(m_selectedDimIndex);
			m_selectedDimIndex = -1;
			changed = true;
		}
		else if (m_selectedNoteIndex >= 0 && m_selectedNoteIndex < m_notes.size())
		{
			if (!isLayerEditable(m_notes[m_selectedNoteIndex].layerId))
			{
				emit statusMessage(QStringLiteral("图层已锁定，无法删除"));
				event->accept();
				return;
			}
			m_notes.removeAt(m_selectedNoteIndex);
			m_selectedNoteIndex = -1;
			changed = true;
		}
		else if (m_selectedSketchId >= 0)
		{
			if (!isLayerEditable(m_sketch.layerOf(m_selectedSketchId)))
			{
				emit statusMessage(QStringLiteral("图层已锁定，无法删除"));
				event->accept();
				return;
			}
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
	root.insert(QStringLiteral("version"), 6);
	root.insert(QStringLiteral("backendId"), m_backendId);
	root.insert(QStringLiteral("projection"),
				m_projection == DrawingProjectionMethod::ThirdAngle ? QStringLiteral("thirdAngle")
																	: QStringLiteral("firstAngle"));
	root.insert(QStringLiteral("currentLayerId"), m_currentLayerId);
	QJsonArray layersArr;
	for (const SheetLayer& L : m_layers)
	{
		QJsonObject lo;
		lo.insert(QStringLiteral("id"), L.id);
		lo.insert(QStringLiteral("name"), L.name);
		lo.insert(QStringLiteral("visible"), L.visible);
		lo.insert(QStringLiteral("locked"), L.locked);
		lo.insert(QStringLiteral("color"), L.color.name(QColor::HexRgb));
		layersArr.append(lo);
	}
	root.insert(QStringLiteral("layers"), layersArr);
	QJsonObject paper;
	QString sizeStr = QStringLiteral("A4");
	switch (m_paper.size)
	{
	case DrawingPaperSize::A3:
		sizeStr = QStringLiteral("A3");
		break;
	case DrawingPaperSize::A2:
		sizeStr = QStringLiteral("A2");
		break;
	case DrawingPaperSize::A1:
		sizeStr = QStringLiteral("A1");
		break;
	case DrawingPaperSize::A0:
		sizeStr = QStringLiteral("A0");
		break;
	case DrawingPaperSize::Custom:
		sizeStr = QStringLiteral("custom");
		break;
	case DrawingPaperSize::A4:
	default:
		sizeStr = QStringLiteral("A4");
		break;
	}
	paper.insert(QStringLiteral("size"), sizeStr);
	paper.insert(QStringLiteral("landscape"), m_paper.landscape);
	paper.insert(QStringLiteral("customWidthMm"), m_paper.customWidthMm);
	paper.insert(QStringLiteral("customHeightMm"), m_paper.customHeightMm);
	paper.insert(QStringLiteral("sheetScale"), m_paper.sheetScale);
	paper.insert(QStringLiteral("title"), m_paper.title);
	paper.insert(QStringLiteral("scaleText"), m_paper.scaleText);
	paper.insert(QStringLiteral("date"), m_paper.date);
	paper.insert(QStringLiteral("visible"), m_paper.visible);
	root.insert(QStringLiteral("paper"), paper);
	QJsonArray viewsArr;
	for (const DrawingView& v : m_views)
	{
		QJsonObject vo;
		vo.insert(QStringLiteral("id"), v.id);
		vo.insert(QStringLiteral("title"), v.title);
		vo.insert(QStringLiteral("kind"), v.kind);
		vo.insert(QStringLiteral("parentViewId"), v.parentViewId);
		vo.insert(QStringLiteral("contentScale"), v.contentScale);
		vo.insert(QStringLiteral("layerId"), v.layerId);
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
		else if (d.kind == SheetDimension::Kind::Angle)
			kind = QStringLiteral("angle");
		o.insert(QStringLiteral("kind"), kind);
		o.insert(QStringLiteral("p1"), QJsonArray{d.p1.x(), d.p1.y()});
		o.insert(QStringLiteral("p2"), QJsonArray{d.p2.x(), d.p2.y()});
		o.insert(QStringLiteral("p3"), QJsonArray{d.p3.x(), d.p3.y()});
		o.insert(QStringLiteral("textOffset"), QJsonArray{d.textOffset.x(), d.textOffset.y()});
		o.insert(QStringLiteral("anchorViewId"), d.anchorViewId);
		o.insert(QStringLiteral("layerId"), d.layerId);
		o.insert(QStringLiteral("sketchEntityId"), d.sketchEntityId);
		if (std::isfinite(d.overrideValue))
			o.insert(QStringLiteral("overrideValue"), d.overrideValue);
		dimsArr.append(o);
	}
	root.insert(QStringLiteral("dimensions"), dimsArr);
	QJsonArray notesArr;
	for (const SheetNote& n : m_notes)
	{
		QJsonObject o;
		o.insert(QStringLiteral("id"), n.id);
		o.insert(QStringLiteral("anchor"), QJsonArray{n.anchor.x(), n.anchor.y()});
		o.insert(QStringLiteral("textPos"), QJsonArray{n.textPos.x(), n.textPos.y()});
		o.insert(QStringLiteral("text"), n.text);
		o.insert(QStringLiteral("anchorViewId"), n.anchorViewId);
		o.insert(QStringLiteral("layerId"), n.layerId);
		notesArr.append(o);
	}
	root.insert(QStringLiteral("notes"), notesArr);
	QJsonObject sketchLayers;
	const QHash<int, QString> el = m_sketch.entityLayers();
	for (auto it = el.constBegin(); it != el.constEnd(); ++it)
		sketchLayers.insert(QString::number(it.key()), it.value());
	root.insert(QStringLiteral("sketchLayers"), sketchLayers);
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
	m_notes.clear();
	m_sketch.clear();
	m_layers.clear();
	m_selectedSketchId = -1;
	m_selectedDimIndex = -1;
	m_selectedNoteIndex = -1;
	m_backendId = root.value(QStringLiteral("backendId")).toString();
	const QString proj = root.value(QStringLiteral("projection")).toString();
	m_projection = (proj == QLatin1String("thirdAngle")) ? DrawingProjectionMethod::ThirdAngle
														 : DrawingProjectionMethod::FirstAngle;
	m_nextLayerSeq = 1;
	for (const QJsonValue& lv : root.value(QStringLiteral("layers")).toArray())
	{
		const QJsonObject lo = lv.toObject();
		SheetLayer L;
		L.id = lo.value(QStringLiteral("id")).toString();
		L.name = lo.value(QStringLiteral("name")).toString(L.id);
		L.visible = lo.value(QStringLiteral("visible")).toBool(true);
		L.locked = lo.value(QStringLiteral("locked")).toBool(false);
		const QString colorName = lo.value(QStringLiteral("color")).toString();
		if (!colorName.isEmpty())
			L.color = QColor(colorName);
		if (L.id.isEmpty())
			continue;
		m_layers.push_back(L);
		if (L.id.startsWith(QLatin1String("L")))
		{
			bool ok = false;
			const int n = L.id.mid(1).toInt(&ok);
			if (ok && n >= m_nextLayerSeq)
				m_nextLayerSeq = n + 1;
		}
	}
	ensureDefaultLayer();
	m_currentLayerId = root.value(QStringLiteral("currentLayerId")).toString(defaultLayerId());
	if (layerIndex(m_currentLayerId) < 0)
		m_currentLayerId = defaultLayerId();
	if (root.contains(QStringLiteral("paper")))
	{
		const QJsonObject po = root.value(QStringLiteral("paper")).toObject();
		const QString sz = po.value(QStringLiteral("size")).toString(QStringLiteral("A4"));
		if (sz == QLatin1String("A3"))
			m_paper.size = DrawingPaperSize::A3;
		else if (sz == QLatin1String("A2"))
			m_paper.size = DrawingPaperSize::A2;
		else if (sz == QLatin1String("A1"))
			m_paper.size = DrawingPaperSize::A1;
		else if (sz == QLatin1String("A0"))
			m_paper.size = DrawingPaperSize::A0;
		else if (sz == QLatin1String("custom") || sz == QLatin1String("Custom"))
			m_paper.size = DrawingPaperSize::Custom;
		else
			m_paper.size = DrawingPaperSize::A4;
		m_paper.landscape = po.value(QStringLiteral("landscape")).toBool(true);
		m_paper.customWidthMm = po.value(QStringLiteral("customWidthMm")).toDouble(297.0);
		m_paper.customHeightMm = po.value(QStringLiteral("customHeightMm")).toDouble(210.0);
		m_paper.sheetScale = po.value(QStringLiteral("sheetScale")).toDouble(1.0);
		if (!(m_paper.sheetScale > 1e-12))
			m_paper.sheetScale = 1.0;
		m_paper.title = po.value(QStringLiteral("title")).toString();
		m_paper.scaleText = po.value(QStringLiteral("scaleText")).toString(QStringLiteral("1:1"));
		if (m_paper.scaleText.isEmpty() || !po.contains(QStringLiteral("sheetScale")))
			syncScaleTextFromSheetScale();
		m_paper.date = po.value(QStringLiteral("date")).toString();
		m_paper.visible = po.value(QStringLiteral("visible")).toBool(true);
	}
	for (const QJsonValue& vv : root.value(QStringLiteral("views")).toArray())
	{
		const QJsonObject vo = vv.toObject();
		DrawingView view;
		view.id = vo.value(QStringLiteral("id")).toString();
		view.title = vo.value(QStringLiteral("title")).toString(view.id);
		view.kind = vo.value(QStringLiteral("kind")).toString(QStringLiteral("front"));
		view.parentViewId = vo.value(QStringLiteral("parentViewId")).toString();
		view.contentScale = vo.value(QStringLiteral("contentScale")).toDouble(1.0);
		view.layerId = vo.value(QStringLiteral("layerId")).toString(defaultLayerId());
		if (layerIndex(view.layerId) < 0)
			view.layerId = defaultLayerId();
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
		else if (kind == QLatin1String("angle"))
			dim.kind = SheetDimension::Kind::Angle;
		else
			dim.kind = SheetDimension::Kind::Linear;
		const QJsonArray a = o.value(QStringLiteral("p1")).toArray();
		const QJsonArray b = o.value(QStringLiteral("p2")).toArray();
		const QJsonArray c = o.value(QStringLiteral("p3")).toArray();
		if (a.size() >= 2)
			dim.p1 = QPointF(a.at(0).toDouble(), a.at(1).toDouble());
		if (b.size() >= 2)
			dim.p2 = QPointF(b.at(0).toDouble(), b.at(1).toDouble());
		if (c.size() >= 2)
			dim.p3 = QPointF(c.at(0).toDouble(), c.at(1).toDouble());
		const QJsonArray to = o.value(QStringLiteral("textOffset")).toArray();
		if (to.size() >= 2)
			dim.textOffset = QPointF(to.at(0).toDouble(), to.at(1).toDouble());
		dim.anchorViewId = o.value(QStringLiteral("anchorViewId")).toString();
		dim.layerId = o.value(QStringLiteral("layerId")).toString(defaultLayerId());
		if (layerIndex(dim.layerId) < 0)
			dim.layerId = defaultLayerId();
		dim.sketchEntityId = o.value(QStringLiteral("sketchEntityId")).toInt(-1);
		if (o.contains(QStringLiteral("overrideValue")))
			dim.overrideValue = o.value(QStringLiteral("overrideValue")).toDouble();
		m_dims.push_back(dim);
	}
	for (const QJsonValue& nv : root.value(QStringLiteral("notes")).toArray())
	{
		const QJsonObject o = nv.toObject();
		SheetNote note;
		note.id = o.value(QStringLiteral("id")).toString();
		const QJsonArray a = o.value(QStringLiteral("anchor")).toArray();
		const QJsonArray t = o.value(QStringLiteral("textPos")).toArray();
		if (a.size() >= 2)
			note.anchor = QPointF(a.at(0).toDouble(), a.at(1).toDouble());
		if (t.size() >= 2)
			note.textPos = QPointF(t.at(0).toDouble(), t.at(1).toDouble());
		note.text = o.value(QStringLiteral("text")).toString();
		note.anchorViewId = o.value(QStringLiteral("anchorViewId")).toString();
		note.layerId = o.value(QStringLiteral("layerId")).toString(defaultLayerId());
		if (layerIndex(note.layerId) < 0)
			note.layerId = defaultLayerId();
		m_notes.push_back(note);
	}
	if (root.contains(QStringLiteral("sketch")))
	{
		const QJsonObject so = root.value(QStringLiteral("sketch")).toObject();
		m_sketch.fromJsonUtf8(QJsonDocument(so).toJson(QJsonDocument::Compact));
	}
	QHash<int, QString> sketchMap;
	const QJsonObject sl = root.value(QStringLiteral("sketchLayers")).toObject();
	for (auto it = sl.begin(); it != sl.end(); ++it)
	{
		bool ok = false;
		const int id = it.key().toInt(&ok);
		if (!ok)
			continue;
		QString lid = it.value().toString(defaultLayerId());
		if (layerIndex(lid) < 0)
			lid = defaultLayerId();
		sketchMap.insert(id, lid);
	}
	m_sketch.setEntityLayers(sketchMap);
	m_needInitialFit = true;
	emit layersChanged();
	emit sheetChanged();
	fitToView();
	update();
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
	const double s = m_paper.sheetScale > 1e-12 ? m_paper.sheetScale : 1.0;
	if (std::abs(s - 1.0) > 1e-12)
	{
		for (Polyline2d& poly : lVis)
			for (QPointF& p : poly.points)
				p = QPointF(p.x() * s, p.y() * s);
		for (Polyline2d& poly : lHid)
			for (QPointF& p : poly.points)
				p = QPointF(p.x() * s, p.y() * s);
		w *= s;
		h *= s;
	}
	DrawingView view;
	view.id = QStringLiteral("%1_%2").arg(kind).arg(m_nextCatalogViewId++);
	view.title = found->title.isEmpty() ? kind : found->title;
	view.kind = kind;
	view.visible = offsetPolylines(lVis, sceneTopLeft);
	view.hidden = offsetPolylines(lHid, sceneTopLeft);
	view.frame = QRectF(sceneTopLeft.x(), sceneTopLeft.y(), w, h).adjusted(-8, -18, 8, 8);
	view.layerId = m_currentLayerId;
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
